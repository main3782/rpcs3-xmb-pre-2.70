#include "stdafx.h"
#include "Emu/Cell/PPUThread.h"
#include "Emu/Memory/vm.h"

#include "sys_btsetting.h"
#include "Emu/Cell/ErrorCodes.h"

LOG_CHANNEL(sys_btsetting);

error_code sys_btsetting_if(u64 cmd, vm::ptr<void> msg)
{
	sys_btsetting.todo("sys_btsetting_if(cmd=0x%llx, msg=*0x%x)", cmd, msg);

	return CELL_OK;
}

// sys_usbbtaudio_initialize (syscall 640, deprecated)
//
// VSH 1.02's libusbbtaudio proxy (UsbBtAudioProxy.cc) calls this from ScePafJob and then asserts "m_pShm != NULL", which aborts VSH
// (sys_process_exit(1), i.e. the XMB window just closes). Observed arguments:
//   r3 = event queue id, r4 = pointer into VSH's own data (out parameter, receives the shared memory address -- assumption),
//   r5 = 0, r6 = 500, r7 = 0x1000, r8 = 0, r9 = 0xffffffff, r10 = 0
// The shared memory layout is unknown, so hand out a zero-filled block and log enough (caller code, out memory) to correct the
// guess from the next log if the proxy needs more.
error_code sys_usbbtaudio_initialize(ppu_thread& ppu)
{
	const u32 equeue_id = static_cast<u32>(ppu.gpr[3]);
	const u32 out = static_cast<u32>(ppu.gpr[4]);

	sys_btsetting.warning("sys_usbbtaudio_initialize(equeue_id=0x%x, out=*0x%x, r5=0x%x, r6=0x%x, r7=0x%x, r8=0x%x, r9=0x%x, r10=0x%x)",
		equeue_id, out, ppu.gpr[5], ppu.gpr[6], ppu.gpr[7], ppu.gpr[8], ppu.gpr[9], ppu.gpr[10]);

	// Diagnostic: code around the syscall in the caller
	{
		const u32 cia = static_cast<u32>(ppu.cia);
		const u32 first = cia - 0x40;

		std::string text;

		for (u32 i = 0; i < 52 && vm::check_addr(first + i * 4, vm::page_readable, 4); i++)
		{
			if (i % 8 == 0)
			{
				fmt::append(text, "\n    %08x:", first + i * 4);
			}

			fmt::append(text, " %08x", static_cast<u32>(vm::_ref<u32>(first + i * 4)));
		}

		sys_btsetting.warning("VSH diag: sys_usbbtaudio_initialize caller code (sc at 0x%x):%s", cia, text);
	}

	static atomic_t<u32> s_shm = 0;

	if (!s_shm)
	{
		const u32 addr = vm::alloc(0x10000, vm::main);

		if (!addr)
		{
			return CELL_ENOMEM;
		}

		std::memset(vm::base(addr), 0, 0x10000);
		s_shm.compare_and_swap(0, addr);
	}

	const u32 shm = s_shm;

	if (out >= 0x10000 && vm::check_addr(out, vm::page_writable, 4))
	{
		std::string before;

		for (u32 i = 0; i < 8 && vm::check_addr(out + i * 4, vm::page_readable, 4); i++)
		{
			fmt::append(before, " %08x", static_cast<u32>(vm::_ref<u32>(out + i * 4)));
		}

		vm::write32(out, shm);

		sys_btsetting.warning("VSH diag: sys_usbbtaudio_initialize: stored shared memory 0x%x at *0x%x (before:%s)", shm, out, before);
	}

	return CELL_OK;
}
