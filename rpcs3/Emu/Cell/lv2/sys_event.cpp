#include "stdafx.h"
#include "sys_event.h"

#include "Emu/IdManager.h"
#include "Emu/IPC.h"
#include "Emu/System.h"
#include "Emu/system_config.h"

#include "Emu/Cell/ErrorCodes.h"
#include "Emu/Memory/vm.h"
#include "Emu/Cell/PPUThread.h"
#include "Emu/Cell/SPUThread.h"
#include "Emu/Cell/timers.hpp"
#include "sys_process.h"

#include "util/asm.hpp"

LOG_CHANNEL(sys_event);

lv2_event_queue::lv2_event_queue(u32 protocol, s32 type, s32 size, u64 name, u64 ipc_key) noexcept
	: id(idm::last_id<lv2_event_queue>())
	, protocol{static_cast<u8>(protocol)}
	, type(static_cast<u8>(type))
	, size(static_cast<u8>(size))
	, name(name)
	, key(ipc_key)
{
}

lv2_event_queue::lv2_event_queue(utils::serial& ar) noexcept
	: id(idm::last_id<lv2_event_queue>())
	, protocol(ar)
	, type(ar)
	, size(ar)
	, name(ar)
	, key(ar)
{
	ar(events);
}

std::function<void(void*)> lv2_event_queue::load(utils::serial& ar)
{
	auto queue = make_shared<lv2_event_queue>(stx::exact_t<utils::serial&>(ar));
	return [ptr = lv2_obj::load(queue->key, queue)](void* storage) { *static_cast<atomic_ptr<lv2_obj>*>(storage) = ptr; };
}

void lv2_event_queue::save(utils::serial& ar)
{
	ar(protocol, type, size, name, key, events);
}

void lv2_event_queue::save_ptr(utils::serial& ar, lv2_event_queue* q)
{
	if (!lv2_obj::check(q))
	{
		ar(u32{0});
		return;
	}

	ar(q->id);
}

shared_ptr<lv2_event_queue> lv2_event_queue::load_ptr(utils::serial& ar, shared_ptr<lv2_event_queue>& queue, std::string_view msg)
{
	const u32 id = ar.pop<u32>();

	if (!id)
	{
		return {};
	}

	if (auto q = idm::get_unlocked<lv2_obj, lv2_event_queue>(id))
	{
		// Already initialized
		return q;
	}

	if (id >> 24 != id_base >> 24)
	{
		fmt::throw_exception("Failed in event queue pointer deserialization (invalid ID): location: %s, id=0x%x", msg, id);
	}

	Emu.PostponeInitCode([id, &queue, msg_str = std::string{msg}]()
	{
		// Defer resolving
		queue = idm::get_unlocked<lv2_obj, lv2_event_queue>(id);

		if (!queue)
		{
			fmt::throw_exception("Failed in event queue pointer deserialization (not found): location: %s, id=0x%x", msg_str, id);
		}
	});

	// Null until resolved
	return {};
}

lv2_event_port::lv2_event_port(utils::serial& ar)
	: type(ar)
	, name(ar)
	, queue(lv2_event_queue::load_ptr(ar, queue, "eventport"))
{
}

void lv2_event_port::save(utils::serial& ar)
{
	ar(type, name);

	lv2_event_queue::save_ptr(ar, queue.get());
}

shared_ptr<lv2_event_queue> lv2_event_queue::find(u64 ipc_key)
{
	if (ipc_key == SYS_EVENT_QUEUE_LOCAL)
	{
		// Invalid IPC key
		return {};
	}

	return g_fxo->get<ipc_manager<lv2_event_queue, u64>>().get(ipc_key);
}

extern void resume_spu_thread_group_from_waiting(spu_thread& spu, std::array<shared_ptr<named_thread<spu_thread>>, 8>& notify_spus);

// Diagnostic (2.17 VSH bring-up): IPC key of the audio server queue that VSH talks to
static constexpr u64 s_vsh_audio_ipc_key = 0x8000cafe0246fab4;

// Diagnostic: describe where in guest code the current PPU thread is (LR + a few saved return addresses)
static std::string event_caller_trace()
{
	const auto ppu = cpu_thread::get_current<ppu_thread>();

	if (!ppu)
	{
		return "(not a PPU thread)";
	}

	std::string out = fmt::format("lr=0x%x bt=[", static_cast<u32>(ppu->lr));
	u32 sp = static_cast<u32>(ppu->gpr[1]);

	for (u32 i = 0; i < 6; i++)
	{
		if (!vm::check_addr(sp, vm::page_readable, 8))
		{
			break;
		}

		const u32 back = static_cast<u32>(vm::_ref<u64>(sp));

		if (back <= sp || !vm::check_addr(back + 0x10, vm::page_readable, 8))
		{
			break;
		}

		fmt::append(out, "%s0x%x", i ? " " : "", static_cast<u32>(vm::_ref<u64>(back + 0x10)));
		sp = back;
	}

	out += "]";
	return out;
}

CellError lv2_event_queue::send(lv2_event event, bool* notified_thread, lv2_event_port* port)
{
	if (notified_thread)
	{
		*notified_thread = false;
	}

	struct notify_spus_t 
	{
		std::array<shared_ptr<named_thread<spu_thread>>, 8> spus;

		~notify_spus_t() noexcept
		{
			for (auto& spu : spus)
			{
				if (spu && spu->state & cpu_flag::wait)
				{
					spu->state.notify_one();
				}
			}
		}
	} notify_spus{};

	std::lock_guard lock(mutex);

	if (!exists)
	{
		return CELL_ENOTCONN;
	}

	if (!pq && !sq)
	{
		if (events.size() < this->size + 0u)
		{
			// Save event
			events.emplace_back(event);
			return {};
		}

		return CELL_EBUSY;
	}

	if (type == SYS_PPU_QUEUE)
	{
		// Store event in registers
		auto& ppu = static_cast<ppu_thread&>(*schedule<ppu_thread>(pq, protocol));

		if (ppu.state & cpu_flag::again)
		{
			if (auto cpu = get_current_cpu_thread())
			{
				cpu->state += cpu_flag::again + cpu_flag::exit;
			}

			sys_event.warning("Ignored event!");

			// Fake error for abort
			return CELL_EAGAIN;
		}

		std::tie(ppu.gpr[4], ppu.gpr[5], ppu.gpr[6], ppu.gpr[7]) = event;

		if (const u32 delay = g_cfg.core.ppu_event_wakeup_delay; delay && cpu_thread::get_current<spu_thread>())
		{
			// measured about 19us from the SPU mailbox write to the PPU receive return on real hardware
			ppu.event_wakeup_time = get_system_time() + delay;
		}

		awake(&ppu);

		if (port && ppu.prio.load().prio < ensure(cpu_thread::get_current<ppu_thread>())->prio.load().prio)
		{
			// Block event port disconnection for the time being of sending events
			// PPU -> lower prio PPU is the only case that can cause thread blocking 
			port->is_busy++;
			ensure(notified_thread);
			*notified_thread = true;
		}
	}
	else
	{
		// Store event in In_MBox
		auto& spu = static_cast<spu_thread&>(*schedule<spu_thread>(sq, protocol));

		if (spu.state & cpu_flag::again)
		{
			if (auto cpu = get_current_cpu_thread())
			{
				cpu->state += cpu_flag::exit + cpu_flag::again;
			}

			sys_event.warning("Ignored event!");

			// Fake error for abort
			return CELL_EAGAIN;
		}

		const u32 data1 = static_cast<u32>(std::get<1>(event));
		const u32 data2 = static_cast<u32>(std::get<2>(event));
		const u32 data3 = static_cast<u32>(std::get<3>(event));
		spu.ch_in_mbox.set_values(4, CELL_OK, data1, data2, data3);
		resume_spu_thread_group_from_waiting(spu, notify_spus.spus);
	}

	return {};
}

error_code sys_event_queue_create(cpu_thread& cpu, vm::ptr<u32> equeue_id, vm::ptr<sys_event_queue_attribute_t> attr, u64 ipc_key, s32 size)
{
	cpu.state += cpu_flag::wait;

	sys_event.warning("sys_event_queue_create(equeue_id=*0x%x, attr=*0x%x, ipc_key=0x%llx, size=%d)", equeue_id, attr, ipc_key, size);

	if (size <= 0 || size > 127)
	{
		return CELL_EINVAL;
	}

	const u32 protocol = attr->protocol;

	if (protocol != SYS_SYNC_FIFO && protocol != SYS_SYNC_PRIORITY)
	{
		sys_event.error("sys_event_queue_create(): unknown protocol (0x%x)", protocol);
		return CELL_EINVAL;
	}

	const u32 type = attr->type;

	if (type != SYS_PPU_QUEUE && type != SYS_SPU_QUEUE)
	{
		sys_event.error("sys_event_queue_create(): unknown type (0x%x)", type);
		return CELL_EINVAL;
	}

	const u32 pshared = ipc_key == SYS_EVENT_QUEUE_LOCAL ? SYS_SYNC_NOT_PROCESS_SHARED : SYS_SYNC_PROCESS_SHARED;
	constexpr u32 flags = SYS_SYNC_NEWLY_CREATED;
	const u64 name = attr->name_u64;

	if (const auto error = lv2_obj::create<lv2_event_queue>(pshared, ipc_key, flags, [&]()
	{
		return make_shared<lv2_event_queue>(protocol, type, size, name, ipc_key);
	}))
	{
		return error;
	}

	cpu.check_state();
	*equeue_id = idm::last_id<lv2_event_queue>();
	return CELL_OK;
}

error_code sys_event_queue_destroy(ppu_thread& ppu, u32 equeue_id, s32 mode)
{
	ppu.state += cpu_flag::wait;

	sys_event.warning("sys_event_queue_destroy(equeue_id=0x%x, mode=%d)", equeue_id, mode);

	if (mode && mode != SYS_EVENT_QUEUE_DESTROY_FORCE)
	{
		return CELL_EINVAL;
	}

	struct notify_spus_t 
	{
		std::array<shared_ptr<named_thread<spu_thread>>, 8> spus;

		~notify_spus_t() noexcept
		{
			for (auto& spu : spus)
			{
				if (spu && spu->state & cpu_flag::wait)
				{
					spu->state.notify_one();
				}
			}
		}
	} notify_spus{};

	std::vector<lv2_event> events;

	std::unique_lock<shared_mutex> qlock;

	cpu_thread* head{};

	const auto queue = idm::withdraw<lv2_obj, lv2_event_queue>(equeue_id, [&](lv2_event_queue& queue) -> CellError
	{
		qlock = std::unique_lock{queue.mutex};

		head = queue.type == SYS_PPU_QUEUE ? static_cast<cpu_thread*>(+queue.pq) : +queue.sq;

		if (!mode && head)
		{
			return CELL_EBUSY;
		}

		for (auto cpu = head; cpu; cpu = cpu->get_next_cpu())
		{
			if (cpu->state & cpu_flag::again)
			{
				ppu.state += cpu_flag::again;
				return CELL_EAGAIN;
			}
		}

		if (!queue.events.empty())
		{
			// Copy events for logging, does not empty
			events.insert(events.begin(), queue.events.begin(), queue.events.end());
		}

		lv2_obj::on_id_destroy(queue, queue.key);

		if (!head)
		{
			qlock.unlock();
		}

		return {};
	});

	if (!queue)
	{
		return CELL_ESRCH;
	}

	if (ppu.state & cpu_flag::again)
	{
		return {};
	}

	if (queue.ret)
	{
		return queue.ret;
	}

	std::string lost_data;

	if (qlock.owns_lock())
	{
		if (sys_event.warning)
		{
			u32 size = 0;

			for (auto cpu = head; cpu; cpu = cpu->get_next_cpu())
			{
				size++;
			}

			fmt::append(lost_data, "Forcefully awaken waiters (%u):\n", size);

			for (auto cpu = head; cpu; cpu = cpu->get_next_cpu())
			{
				lost_data += cpu->get_name();
				lost_data += '\n';
			}
		}

		if (queue->type == SYS_PPU_QUEUE)
		{
			for (auto cpu = +queue->pq; cpu; cpu = cpu->next_cpu)
			{
				cpu->gpr[3] = CELL_ECANCELED;
				queue->append(cpu);
			}

			atomic_storage<ppu_thread*>::release(queue->pq, nullptr);
			lv2_obj::awake_all();
		}
		else
		{
			for (auto cpu = +queue->sq; cpu; cpu = cpu->next_cpu)
			{
				cpu->ch_in_mbox.set_values(1, CELL_ECANCELED);
				resume_spu_thread_group_from_waiting(*cpu, notify_spus.spus);
			}

			atomic_storage<spu_thread*>::release(queue->sq, nullptr);
		}

		qlock.unlock();
	}

	if (sys_event.warning)
	{
		if (!events.empty())
		{
			fmt::append(lost_data, "Unread queue events (%u):\n", events.size());
		}

		for (const lv2_event& evt : events)
		{
			fmt::append(lost_data, "data0=0x%x, data1=0x%x, data2=0x%x, data3=0x%x\n"
				, std::get<0>(evt), std::get<1>(evt), std::get<2>(evt), std::get<3>(evt));
		}

		if (!lost_data.empty())
		{
			sys_event.warning("sys_event_queue_destroy(): %s", lost_data);
		}
	}

	return CELL_OK;
}

error_code sys_event_queue_tryreceive(ppu_thread& ppu, u32 equeue_id, vm::ptr<sys_event_t> event_array, s32 size, vm::ptr<u32> number)
{
	ppu.state += cpu_flag::wait;

	sys_event.trace("sys_event_queue_tryreceive(equeue_id=0x%x, event_array=*0x%x, size=%d, number=*0x%x)", equeue_id, event_array, size, number);

	const auto queue = idm::get_unlocked<lv2_obj, lv2_event_queue>(equeue_id);

	if (!queue)
	{
		return CELL_ESRCH;
	}

	if (queue->type != SYS_PPU_QUEUE)
	{
		return CELL_EINVAL;
	}

	std::array<sys_event_t, 127> events;

	std::unique_lock lock(queue->mutex);

	if (!queue->exists)
	{
		return CELL_ESRCH;
	}

	s32 count = 0;

	while (count < size && !queue->events.empty())
	{
		auto& dest = events[count++];
		std::tie(dest.source, dest.data1, dest.data2, dest.data3) = queue->events.front();
		queue->events.pop_front();
	}

	lock.unlock();
	ppu.check_state();

	std::copy_n(events.begin(), count, event_array.get_ptr());
	*number = count;

	return CELL_OK;
}

error_code sys_event_queue_receive(ppu_thread& ppu, u32 equeue_id, vm::ptr<sys_event_t> dummy_event, u64 timeout)
{
	ppu.state += cpu_flag::wait;

	sys_event.trace("sys_event_queue_receive(equeue_id=0x%x, *0x%x, timeout=0x%llx)", equeue_id, dummy_event, timeout);

	ppu.gpr[3] = CELL_OK;

	const auto queue = idm::get<lv2_obj, lv2_event_queue>(equeue_id, [&, notify = lv2_obj::notify_all_t()](lv2_event_queue& queue) -> CellError
	{
		if (queue.type != SYS_PPU_QUEUE)
		{
			return CELL_EINVAL;
		}

		lv2_obj::prepare_for_sleep(ppu);

		std::lock_guard lock(queue.mutex);

		// "/dev_flash/vsh/module/msmw2.sprx" seems to rely on some cryptic shared memory behaviour that we don't emulate correctly
		// This is a hack to avoid waiting for 1m40s every time we boot vsh
		if (queue.key == 0x8005911000000012 && Emu.IsVsh())
		{
			sys_event.todo("sys_event_queue_receive(equeue_id=0x%x, *0x%x, timeout=0x%llx) Bypassing timeout for msmw2.sprx", equeue_id, dummy_event, timeout);
			timeout = 1;
		}

		if (queue.events.empty())
		{
			queue.sleep(ppu, timeout);
			lv2_obj::emplace(queue.pq, &ppu);
			return CELL_EBUSY;
		}

		std::tie(ppu.gpr[4], ppu.gpr[5], ppu.gpr[6], ppu.gpr[7]) = queue.events.front();
		queue.events.pop_front();
		return {};
	});

	if (!queue)
	{
		return CELL_ESRCH;
	}

	if (queue.ret)
	{
		if (queue.ret != CELL_EBUSY)
		{
			return queue.ret;
		}
	}
	else
	{
		return CELL_OK;
	}

	// If cancelled, gpr[3] will be non-zero. Other registers must contain event data.
	while (auto state = +ppu.state)
	{
		if (state & cpu_flag::signal && ppu.state.test_and_reset(cpu_flag::signal))
		{
			break;
		}

		if (is_stopped(state))
		{
			std::lock_guard lock_rsx(queue->mutex);

			for (auto cpu = +queue->pq; cpu; cpu = cpu->next_cpu)
			{
				if (cpu == &ppu)
				{
					ppu.state += cpu_flag::again;
					return {};
				}
			}

			break;
		}

		for (usz i = 0; cpu_flag::signal - ppu.state && i < 50; i++)
		{
			busy_wait(500);
		}

		if (ppu.state & cpu_flag::signal)
 		{
			continue;
		}

		if (timeout)
		{
			if (lv2_obj::wait_timeout(timeout, &ppu))
			{
				// Wait for rescheduling
				if (ppu.check_state())
				{
					continue;
				}

				ppu.state += cpu_flag::wait;

				if (!atomic_storage<ppu_thread*>::load(queue->pq))
				{
					// Waiters queue is empty, so the thread must have been signaled
					queue->mutex.lock_unlock();
					break;
				}

				std::lock_guard lock(queue->mutex);

				if (!queue->unqueue(queue->pq, &ppu))
				{
					break;
				}

				ppu.gpr[3] = CELL_ETIMEDOUT;
				break;
			}
		}
		else
		{
			ppu.state.wait(state);
		}
	}

	if (const u64 wakeup_time = std::exchange(ppu.event_wakeup_time, 0))
	{
		if (const u64 current = get_system_time(); current < wakeup_time)
		{
			thread_ctrl::wait_for_accurate(wakeup_time - current);
		}
	}

	return not_an_error(ppu.gpr[3]);
}

error_code sys_event_queue_drain(ppu_thread& ppu, u32 equeue_id)
{
	ppu.state += cpu_flag::wait;

	sys_event.trace("sys_event_queue_drain(equeue_id=0x%x)", equeue_id);

	const auto queue = idm::check<lv2_obj, lv2_event_queue>(equeue_id, [&](lv2_event_queue& queue)
	{
		std::lock_guard lock(queue.mutex);

		queue.events.clear();
	});

	if (!queue)
	{
		return CELL_ESRCH;
	}

	return CELL_OK;
}

error_code sys_event_port_create(cpu_thread& cpu, vm::ptr<u32> eport_id, s32 port_type, u64 name)
{
	cpu.state += cpu_flag::wait;

	sys_event.warning("sys_event_port_create(eport_id=*0x%x, port_type=%d, name=0x%llx)", eport_id, port_type, name);

	if (port_type != SYS_EVENT_PORT_LOCAL && port_type != SYS_EVENT_PORT_IPC)
	{
		sys_event.error("sys_event_port_create(): unknown port type (%d)", port_type);
		return CELL_EINVAL;
	}

	if (const u32 id = idm::make<lv2_obj, lv2_event_port>(port_type, name))
	{
		cpu.check_state();
		*eport_id = id;
		return CELL_OK;
	}

	return CELL_EAGAIN;
}

error_code sys_event_port_destroy(ppu_thread& ppu, u32 eport_id)
{
	ppu.state += cpu_flag::wait;

	sys_event.warning("sys_event_port_destroy(eport_id=0x%x)", eport_id);

	const auto port = idm::withdraw<lv2_obj, lv2_event_port>(eport_id, [](lv2_event_port& port) -> CellError
	{
		if (lv2_obj::check(port.queue))
		{
			return CELL_EISCONN;
		}

		return {};
	});

	if (!port)
	{
		return CELL_ESRCH;
	}

	if (port.ret)
	{
		return port.ret;
	}

	return CELL_OK;
}

error_code sys_event_port_connect_local(cpu_thread& cpu, u32 eport_id, u32 equeue_id)
{
	cpu.state += cpu_flag::wait;

	sys_event.warning("sys_event_port_connect_local(eport_id=0x%x, equeue_id=0x%x)", eport_id, equeue_id);

	std::lock_guard lock(id_manager::g_mutex);

	const auto port = idm::check_unlocked<lv2_obj, lv2_event_port>(eport_id);
	auto queue = idm::get_unlocked<lv2_obj, lv2_event_queue>(equeue_id);

	if (!port || !queue)
	{
		return CELL_ESRCH;
	}

	if (port->type != SYS_EVENT_PORT_LOCAL)
	{
		return CELL_EINVAL;
	}

	if (lv2_obj::check(port->queue))
	{
		return CELL_EISCONN;
	}

	port->queue = std::move(queue);

	return CELL_OK;
}

error_code sys_event_port_connect_ipc(ppu_thread& ppu, u32 eport_id, u64 ipc_key)
{
	ppu.state += cpu_flag::wait;

	sys_event.warning("sys_event_port_connect_ipc(eport_id=0x%x, ipc_key=0x%x)", eport_id, ipc_key);

	if (ipc_key == 0)
	{
		return CELL_EINVAL;
	}

	auto queue = lv2_event_queue::find(ipc_key);

	// Older firmware (e.g. 2.17) runs sys_audio.self as a separate process that owns this queue,
	// and vshmw polls it forever ("waiting for audio server process ready") until it exists.
	// We can't spawn that process, so provide an empty placeholder queue to get VSH past the wait.
	// Must be done before taking id_manager::g_mutex (lv2_obj::create locks it internally).
	if (!queue && ipc_key == 0x8000cafe0246fab4 && Emu.IsVsh())
	{
		sys_event.todo("sys_event_port_connect_ipc(): creating placeholder queue for the audio server (ipc_key=0x%llx)", ipc_key);

		if (!lv2_obj::create<lv2_event_queue>(SYS_SYNC_PROCESS_SHARED, ipc_key, SYS_SYNC_NEWLY_CREATED, [&]()
		{
			return make_shared<lv2_event_queue>(SYS_SYNC_FIFO, SYS_PPU_QUEUE, 127, 0, ipc_key);
		}))
		{
			queue = lv2_event_queue::find(ipc_key);
		}
	}

	std::lock_guard lock(id_manager::g_mutex);

	const auto port = idm::check_unlocked<lv2_obj, lv2_event_port>(eport_id);

	if (!port || !queue)
	{
		return CELL_ESRCH;
	}

	if (port->type != SYS_EVENT_PORT_IPC)
	{
		return CELL_EINVAL;
	}

	if (lv2_obj::check(port->queue))
	{
		return CELL_EISCONN;
	}

	port->queue = std::move(queue);

	return CELL_OK;
}

error_code sys_event_port_disconnect(ppu_thread& ppu, u32 eport_id)
{
	ppu.state += cpu_flag::wait;

	sys_event.warning("sys_event_port_disconnect(eport_id=0x%x)", eport_id);

	std::lock_guard lock(id_manager::g_mutex);

	const auto port = idm::check_unlocked<lv2_obj, lv2_event_port>(eport_id);

	if (!port)
	{
		return CELL_ESRCH;
	}

	if (!lv2_obj::check(port->queue))
	{
		return CELL_ENOTCONN;
	}

	if (port->is_busy)
	{
		return CELL_EBUSY;
	}

	port->queue.reset();

	return CELL_OK;
}

// Minimal emulation of the PS3 audio server (sys_audio.self) for 2.17-era VSH.
// The client side (vshmw, libaudio init at 0x31ce70..0x31d1e8) performs this handshake over IPC queues:
//   request d1 = opcode in the low 9 bits (session id in the upper bits), d2 = reply queue ipc key
//   reply   d1 = reply opcode (same upper bits), d2/d3 = payload
//   op 0x01 -> reply 0x02, d2 = session id
//   op 0x30 -> reply 0x30, d2 = generation (must differ from the client's cached value, initially 0)
//   op 0x07 -> reply 0x08, d2 = shared memory size, d3 = shared memory flags (page size)
//   op 0x09 -> reply 0x0a, d2 = shared memory ipc key, d3 = low 16 bits stored by the client (count)
//   op 0x1c -> reply 0x1d, d2 = offset into the shared memory of the parameter block
// Only after all of them succeeded does the client set its "initialized" flag (otherwise it returns
// CELL_AUDIO_ERROR_NOT_INIT, which paf then stores and uses to skip display setup).
static void vsh_audio_server_request(u64 d1, u64 d2, u64 d3)
{
	static atomic_t<u32> s_log_count = 0;
	static atomic_t<u64> s_reply_key = 0;

	const bool do_log = s_log_count++ < 200;
	const u32 op = static_cast<u32>(d1 & 0x1ff);

	// The reply queue key is only guaranteed to be valid in the first request, remember it
	if (op == 0x01)
	{
		s_reply_key = d2;
	}

	const u64 reply_key = op == 0x01 ? d2 : s_reply_key.load();

	if (do_log)
	{
		sys_event.warning("AUDIO-SRV request: op=0x%x d1=0x%llx d2=0x%llx d3=0x%llx reply_key=0x%llx %s", op, d1, d2, d3, reply_key, event_caller_trace());
	}

	u64 reply_op = 0;
	u64 r2 = 0;
	u64 r3 = 0;

	switch (op)
	{
	case 0x01: reply_op = 0x02; r2 = 0x200; break;
	case 0x30: reply_op = 0x30; r2 = 1; break;
	case 0x07: reply_op = 0x08; r2 = 0x100000; r3 = 0x400; break;
	case 0x09: reply_op = 0x0a; r2 = 0x8000cafe02465a00ull; r3 = 8; break;
	case 0x1c: reply_op = 0x1d; r2 = 0; break;
	default:
	{
		sys_event.error("AUDIO-SRV: unknown request op=0x%x (d1=0x%llx), not replying", op, d1);
		return;
	}
	}

	const auto reply_queue = lv2_event_queue::find(reply_key);

	if (!reply_queue)
	{
		sys_event.error("AUDIO-SRV: reply queue 0x%llx not found", reply_key);
		return;
	}

	const u64 reply_d1 = (d1 & ~u64{0x1ff} & ~(u64{1} << 63)) | reply_op;

	if (do_log)
	{
		sys_event.warning("AUDIO-SRV reply: d1=0x%llx d2=0x%llx d3=0x%llx", reply_d1, r2, r3);
	}

	reply_queue->send((u64{1} << 32) | 1, reply_d1, r2, r3);
}

error_code sys_event_port_send(u32 eport_id, u64 data1, u64 data2, u64 data3)
{
	const auto cpu = cpu_thread::get_current();
	const auto ppu = cpu ? cpu->try_get<ppu_thread>() : nullptr;

	if (cpu)
	{
		cpu->state += cpu_flag::wait;
	}

	sys_event.trace("sys_event_port_send(eport_id=0x%x, data1=0x%llx, data2=0x%llx, data3=0x%llx)", eport_id, data1, data2, data3);

	bool notified_thread = false;

	const auto port = idm::check<lv2_obj, lv2_event_port>(eport_id, [&, notify = lv2_obj::notify_all_t()](lv2_event_port& port) -> CellError
	{
		if (ppu && ppu->loaded_from_savestate)
		{
			port.is_busy++;
			notified_thread = true;
			return {};
		}

		if (lv2_obj::check(port.queue))
		{
			if (port.queue->key == s_vsh_audio_ipc_key && Emu.IsVsh())
			{
				// Fake audio server answers instead of enqueueing into the placeholder queue
				vsh_audio_server_request(data1, data2, data3);
				return {};
			}

			const u64 source = port.name ? port.name : (u64{process_getpid() + 0u} << 32) | u64{eport_id};

			return port.queue->send(source, data1, data2, data3, &notified_thread, ppu && port.queue->type == SYS_PPU_QUEUE ? &port : nullptr);
		}

		return CELL_ENOTCONN;
	});

	if (!port)
	{
		return CELL_ESRCH;
	}

	if (ppu && notified_thread)
	{
		// Wait to be requeued
		if (ppu->test_stopped())
		{
			// Wait again on savestate load
			ppu->state += cpu_flag::again;
		}

		port->is_busy--;
		return CELL_OK;
	}

	if (port.ret)
	{
		if (port.ret == CELL_EAGAIN)
		{
			// Not really an error code exposed to games (thread has raised cpu_flag::again)
			return not_an_error(CELL_EAGAIN);
		}

		if (port.ret == CELL_EBUSY)
		{
			return not_an_error(CELL_EBUSY);
		}

		return port.ret;
	}

	return CELL_OK;
}
