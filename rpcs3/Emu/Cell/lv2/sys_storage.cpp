#include "stdafx.h"

#include <array>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "Emu/IdManager.h"
#include "Emu/Memory/vm.h"

#include "Emu/Cell/ErrorCodes.h"
#include "sys_event.h"
#include "sys_fs.h"
#include "util/shared_ptr.hpp"

#include "sys_storage.h"

LOG_CHANNEL(sys_storage);

namespace
{
	struct storage_manager
	{
		// This is probably wrong and should be assigned per fd or something
		atomic_ptr<lv2_event_queue> asyncequeue;
	};
}

namespace
{
	// Sparse backing store for the emulated internal HDD (every region of it), so that what VSH writes with sys_storage_write
	// while formatting can be read back with sys_storage_read. The 2.17 format/restore routine verifies the filesystem it just wrote
	// ("cg 0: bad magic number" -> assertion -> crash if the readback is all zeros), and its filesystem check on every boot reads it again.
	// Sectors that were never written, or were written as all zeros, read back as zeros.
	// The contents are persisted in an append-only log file (vsh_hdd_sectors.bin in the RPCS3 data directory), so a formatted disk survives
	// restarts of RPCS3. Delete that file to get an unformatted disk again.
	struct hdd_sector_store
	{
		// One log record: device id, sector number, sector data
		static constexpr usz record_size = 8 + 8 + 0x200;

		std::mutex mutex;
		std::unordered_map<u64, std::unordered_map<u64, std::array<u8, 0x200>>> devices; // device id -> sector number -> data
		fs::file log;
		bool loaded = false;

		static bool is_zero(const u8* data)
		{
			static constexpr std::array<u8, 0x200> zero_sector{};
			return std::memcmp(data, zero_sector.data(), 0x200) == 0;
		}

		// Must be called with the mutex held
		void load()
		{
			if (std::exchange(loaded, true))
			{
				return;
			}

			const std::string path = fs::get_config_dir() + "vsh_hdd_sectors.bin";
			usz valid_size = 0;

			if (fs::file in{path})
			{
				std::vector<u8> buf(record_size * 256);

				while (true)
				{
					const u64 got = in.read(buf.data(), buf.size());

					if (got < record_size)
					{
						break;
					}

					for (usz pos = 0; pos + record_size <= got; pos += record_size)
					{
						u64 device = 0, sector = 0;
						std::memcpy(&device, buf.data() + pos, 8);
						std::memcpy(&sector, buf.data() + pos + 8, 8);
						const u8* data = buf.data() + pos + 16;

						if (is_zero(data))
						{
							devices[device].erase(sector);
						}
						else
						{
							std::memcpy(devices[device][sector].data(), data, 0x200);
						}

						valid_size += record_size;
					}

					if (got < buf.size())
					{
						break;
					}
				}

				// Drop a partially written record at the end (e.g. after a crash), so new records stay aligned
				if (valid_size != in.size())
				{
					fs::file fix{path, fs::write};
					fix.trunc(valid_size);
				}

				u64 sector_count = 0;

				for (const auto& dev : devices)
				{
					sector_count += dev.second.size();
				}

				sys_storage.notice("HDD sector store: loaded %u sectors from '%s'", sector_count, path);
			}

			log.open(path, fs::write + fs::create + fs::append);
		}

		// Must be called with the mutex held
		void append_record(std::vector<u8>& out, u64 device, u64 sector, const u8* data)
		{
			const usz pos = out.size();
			out.resize(pos + record_size);
			std::memcpy(out.data() + pos, &device, 8);
			std::memcpy(out.data() + pos + 8, &sector, 8);
			std::memcpy(out.data() + pos + 16, data, 0x200);
		}
	};

	hdd_sector_store& get_hdd_store()
	{
		static hdd_sector_store store;
		return store;
	}

	bool is_emulated_hdd(u64 device)
	{
		return (device & 0xFFFFF00FFFFFFFF) == ATA_HDD;
	}
}

error_code sys_storage_open(u64 device, u64 mode, vm::ptr<u32> fd, u64 flags)
{
	sys_storage.todo("sys_storage_open(device=0x%x, mode=0x%x, fd=*0x%x, flags=0x%x)", device, mode, fd, flags);

	if (device == 0)
	{
		return CELL_ENOENT;
	}

	if (!fd)
	{
		return CELL_EFAULT;
	}

	[[maybe_unused]] u64 storage_id = device & 0xFFFFF00FFFFFFFF;
	fs::file file;

	if (const u32 id = idm::make<lv2_storage>(device, std::move(file), mode, flags))
	{
		*fd = id;
		return CELL_OK;
	}

	return CELL_EAGAIN;
}

error_code sys_storage_close(u32 fd)
{
	sys_storage.todo("sys_storage_close(fd=0x%x)", fd);

	idm::remove<lv2_storage>(fd);

	return CELL_OK;
}

error_code sys_storage_read(u32 fd, u32 mode, u32 start_sector, u32 num_sectors, vm::ptr<void> bounce_buf, vm::ptr<u32> sectors_read, u64 flags)
{
	sys_storage.todo("sys_storage_read(fd=0x%x, mode=0x%x, start_sector=0x%x, num_sectors=0x%x, bounce_buf=*0x%x, sectors_read=*0x%x, flags=0x%x)", fd, mode, start_sector, num_sectors, bounce_buf, sectors_read, flags);

	if (!bounce_buf || !sectors_read)
	{
		return CELL_EFAULT;
	}

	std::memset(bounce_buf.get_ptr(), 0, num_sectors * 0x200ull);
	const auto handle = idm::get_unlocked<lv2_storage>(fd);

	if (!handle)
	{
		return CELL_ESRCH;
	}

	if (is_emulated_hdd(handle->device_id))
	{
		auto& store = get_hdd_store();
		std::lock_guard lock(store.mutex);
		store.load();

		if (const auto dev = store.devices.find(handle->device_id); dev != store.devices.end())
		{
			u8* dst = static_cast<u8*>(bounce_buf.get_ptr());

			for (u32 i = 0; i < num_sectors; i++)
			{
				if (const auto sector = dev->second.find(u64{start_sector} + i); sector != dev->second.end())
				{
					std::memcpy(dst + i * 0x200ull, sector->second.data(), 0x200);
				}
			}
		}
	}

	if (handle->file)
	{
		handle->file.seek(start_sector * 0x200ull);
		const u64 size = num_sectors * 0x200ull;
		const u64 result = lv2_file::op_read(handle->file, bounce_buf, size);
		num_sectors = ::narrow<u32>(result / 0x200ull);
	}

	*sectors_read = num_sectors;

	return CELL_OK;
}

error_code sys_storage_write(u32 fd, u32 mode, u32 start_sector, u32 num_sectors, vm::ptr<void> data, vm::ptr<u32> sectors_wrote, u64 flags)
{
	sys_storage.todo("sys_storage_write(fd=0x%x, mode=0x%x, start_sector=0x%x, num_sectors=0x%x, data=*=0x%x, sectors_wrote=*0x%x, flags=0x%llx)", fd, mode, start_sector, num_sectors, data, sectors_wrote, flags);

	if (!sectors_wrote)
	{
		return CELL_EFAULT;
	}

	const auto handle = idm::get_unlocked<lv2_storage>(fd);

	if (!handle)
	{
		return CELL_ESRCH;
	}

	if (is_emulated_hdd(handle->device_id) && data && num_sectors)
	{
		if (num_sectors > 0x8000 || !vm::check_addr(data.addr(), vm::page_readable, static_cast<u32>(num_sectors * 0x200ull)))
		{
			return CELL_EFAULT;
		}

		const u8* src = static_cast<const u8*>(data.get_ptr());

		auto& store = get_hdd_store();
		std::lock_guard lock(store.mutex);
		store.load();
		auto& dev = store.devices[handle->device_id];

		std::vector<u8> records;

		for (u32 i = 0; i < num_sectors; i++)
		{
			const u8* sector_data = src + i * 0x200ull;
			const u64 sector = u64{start_sector} + i;

			if (hdd_sector_store::is_zero(sector_data))
			{
				// Only a sector that currently holds data needs a record (an all-zero record means "erase")
				if (dev.erase(sector))
				{
					store.append_record(records, handle->device_id, sector, sector_data);
				}
			}
			else
			{
				auto& stored = dev[sector];

				if (std::memcmp(stored.data(), sector_data, 0x200) != 0)
				{
					std::memcpy(stored.data(), sector_data, 0x200);
					store.append_record(records, handle->device_id, sector, sector_data);
				}
			}
		}

		if (!records.empty() && store.log)
		{
			store.log.write(records.data(), records.size());
		}
	}

	*sectors_wrote = num_sectors;

	return CELL_OK;
}

error_code sys_storage_send_device_command(u32 dev_handle, u64 cmd, vm::ptr<void> in, u64 inlen, vm::ptr<void> out, u64 outlen)
{
	sys_storage.todo("sys_storage_send_device_command(dev_handle=0x%x, cmd=0x%llx, in=*0x%x, inlen=0x%x, out=*0x%x, outlen=0x%x)", dev_handle, cmd, in, inlen, out, outlen);

	return CELL_OK;
}

error_code sys_storage_async_configure(u32 fd, u32 io_buf, u32 equeue_id, u32 unk)
{
	sys_storage.todo("sys_storage_async_configure(fd=0x%x, io_buf=0x%x, equeue_id=0x%x, unk=*0x%x)", fd, io_buf, equeue_id, unk);

	auto& manager = g_fxo->get<storage_manager>();

	if (auto queue = idm::get_unlocked<lv2_obj, lv2_event_queue>(equeue_id))
	{
		manager.asyncequeue.store(queue);
	}
	else
	{
		return CELL_ESRCH;
	}

	return CELL_OK;
}

error_code sys_storage_async_send_device_command(u32 dev_handle, u64 cmd, vm::ptr<void> in, u64 inlen, vm::ptr<void> out, u64 outlen, u64 unk)
{
	sys_storage.todo("sys_storage_async_send_device_command(dev_handle=0x%x, cmd=0x%llx, in=*0x%x, inlen=0x%x, out=*0x%x, outlen=0x%x, unk=0x%x)", dev_handle, cmd, in, inlen, out, outlen, unk);

	auto& manager = g_fxo->get<storage_manager>();

	if (auto q = manager.asyncequeue.load())
	{
		q->send(0, unk, unk, unk);
	}

	return CELL_OK;
}

error_code sys_storage_async_read()
{
	sys_storage.todo("sys_storage_async_read()");

	return CELL_OK;
}

error_code sys_storage_async_write()
{
	sys_storage.todo("sys_storage_async_write()");

	return CELL_OK;
}

error_code sys_storage_async_cancel()
{
	sys_storage.todo("sys_storage_async_cancel()");

	return CELL_OK;
}

error_code sys_storage_get_device_info(u64 device, vm::ptr<StorageDeviceInfo> buffer)
{
	sys_storage.todo("sys_storage_get_device_info(device=0x%x, buffer=*0x%x)", device, buffer);

	if (!buffer)
	{
		return CELL_EFAULT;
	}

	memset(buffer.get_ptr(), 0, sizeof(StorageDeviceInfo));

	u64 storage = device & 0xFFFFF00FFFFFFFF;
	u32 dev_num = (device >> 32) & 0xFF;

	if (storage == ATA_HDD) // dev_hdd?
	{
		if (dev_num > 2)
		{
			return not_an_error(-5);
		}

		std::string u = "unnamed";
		memcpy(buffer->name, u.c_str(), u.size());
		buffer->sector_size = 0x200;
		buffer->one = 1;
		buffer->flags[1] = 1;
		buffer->flags[2] = 1;
		buffer->flags[7] = 1;

		// set partition size based on dev_num
		// stole these sizes from kernel dump, unknown if they are 100% correct
		// vsh reports only 2 partitions even though there is 3 sizes
		switch (dev_num)
		{
		case 0:
			buffer->sector_count = 0x2542EAB0; // possibly total size
			break;
		case 1:
			buffer->sector_count = 0x24FAEA98; // which makes this hdd0
			break;
		case 2:
			buffer->sector_count = 0x3FFFF8; // and this one hdd1
			break;
		}
	}
	else if (storage == BDVD_DRIVE) //	dev_bdvd?
	{
		if (dev_num > 0)
		{
			return not_an_error(-5);
		}

		std::string u = "unnamed";
		memcpy(buffer->name, u.c_str(), u.size());
		buffer->sector_count = 0x4D955;
		buffer->sector_size = 0x800;
		buffer->one = 1;
		buffer->flags[1] = 0;
		buffer->flags[2] = 1;
		buffer->flags[7] = 1;
	}
	else if (storage == USB_MASS_STORAGE_1(0))
	{
		if (dev_num > 0)
		{
			return not_an_error(-5);
		}

		std::string u = "unnamed";
		memcpy(buffer->name, u.c_str(), u.size());
		/*buffer->sector_count = 0x4D955;*/
		buffer->sector_size = 0x200;
		buffer->one = 1;
		buffer->flags[1] = 0;
		buffer->flags[2] = 1;
		buffer->flags[7] = 1;
	}
	else if (storage == NAND_FLASH)
	{
		if (dev_num > 6)
		{
			return not_an_error(-5);
		}

		std::string u = "unnamed";
		memcpy(buffer->name, u.c_str(), u.size());
		buffer->sector_size = 0x200;
		buffer->one = 1;
		buffer->flags[1] = 1;
		buffer->flags[2] = 1;
		buffer->flags[7] = 1;

		// see ata_hdd for explanation
		switch (dev_num)
		{
		case 0: buffer->sector_count = 0x80000;
			break;
		case 1: buffer->sector_count = 0x75F8;
			break;
		case 2: buffer->sector_count = 0x63E00;
			break;
		case 3: buffer->sector_count = 0x8000;
			break;
		case 4: buffer->sector_count = 0x400;
			break;
		case 5: buffer->sector_count = 0x2000;
			break;
		case 6: buffer->sector_count = 0x200;
			break;
		}
	}
	else if (storage == NOR_FLASH)
	{
		if (dev_num > 3)
		{
			return not_an_error(-5);
		}

		std::string u = "unnamed";
		memcpy(buffer->name, u.c_str(), u.size());
		buffer->sector_size = 0x200;
		buffer->one = 1;
		buffer->flags[1] = 0;
		buffer->flags[2] = 1;
		buffer->flags[7] = 1;

		// see ata_hdd for explanation
		switch (dev_num)
		{
		case 0: buffer->sector_count = 0x8000;
			break;
		case 1: buffer->sector_count = 0x77F8;
			break;
		case 2: buffer->sector_count = 0x100; // offset, 0x20000
			break;
		case 3: buffer->sector_count = 0x400;
			break;
		}
	}
	else if (storage == NAND_UNK)
	{
		if (dev_num > 1)
		{
			return not_an_error(-5);
		}

		std::string u = "unnamed";
		memcpy(buffer->name, u.c_str(), u.size());
		buffer->sector_size = 0x800;
		buffer->one = 1;
		buffer->flags[1] = 0;
		buffer->flags[2] = 1;
		buffer->flags[7] = 1;

		// see ata_hdd for explanation
		switch (dev_num)
		{
		case 0: buffer->sector_count = 0x7FFFFFFF;
			break;
		}
	}
	else
	{
		sys_storage.error("sys_storage_get_device_info(device=0x%x, buffer=*0x%x)", device, buffer);
	}

	return CELL_OK;
}

error_code sys_storage_get_device_config(vm::ptr<u32> storages, vm::ptr<u32> devices)
{
	sys_storage.todo("sys_storage_get_device_config(storages=*0x%x, devices=*0x%x)", storages, devices);

	if (storages) *storages = 6; else return CELL_EFAULT;
	if (devices)  *devices = 17; else return CELL_EFAULT;

	return CELL_OK;
}

error_code sys_storage_report_devices(u32 storages, u32 start, u32 devices, vm::ptr<u64> device_ids)
{
	sys_storage.todo("sys_storage_report_devices(storages=0x%x, start=0x%x, devices=0x%x, device_ids=0x%x)", storages, start, devices, device_ids);

	if (!device_ids)
	{
		return CELL_EFAULT;
	}

	static constexpr std::array<u64, 0x11> all_devs = []
	{
		std::array<u64, 0x11> all_devs{};
		all_devs[0] = 0x10300000000000A;

		for (int i = 0; i < 7; ++i)
		{
			all_devs[i + 1] = 0x100000000000001 | (static_cast<u64>(i) << 32);
		}

		for (int i = 0; i < 3; ++i)
		{
			all_devs[i + 8] = 0x101000000000007 | (static_cast<u64>(i) << 32);
		}

		all_devs[11] = 0x101000000000006;

		for (int i = 0; i < 4; ++i)
		{
			all_devs[i + 12] = 0x100000000000004 | (static_cast<u64>(i) << 32);
		}

		all_devs[16] = 0x100000000000003;
		return all_devs;
	}();

	if (!devices || start >= all_devs.size() || devices > all_devs.size() - start)
	{
		return CELL_EINVAL;
	}

	std::copy_n(all_devs.begin() + start, devices, device_ids.get_ptr());

	return CELL_OK;
}

error_code sys_storage_configure_medium_event(u32 fd, u32 equeue_id, u32 c)
{
	sys_storage.todo("sys_storage_configure_medium_event(fd=0x%x, equeue_id=0x%x, c=0x%x)", fd, equeue_id, c);

	return CELL_OK;
}

error_code sys_storage_set_medium_polling_interval()
{
	sys_storage.todo("sys_storage_set_medium_polling_interval()");

	return CELL_OK;
}

error_code sys_storage_create_region()
{
	sys_storage.todo("sys_storage_create_region()");

	return CELL_OK;
}

error_code sys_storage_delete_region()
{
	sys_storage.todo("sys_storage_delete_region()");

	return CELL_OK;
}

error_code sys_storage_execute_device_command(u32 fd, u64 cmd, vm::ptr<char> cmdbuf, u64 cmdbuf_size, vm::ptr<char> databuf, u64 databuf_size, vm::ptr<u32> driver_status)
{
	sys_storage.todo("sys_storage_execute_device_command(fd=0x%x, cmd=0x%llx, cmdbuf=*0x%x, cmdbuf_size=0x%llx, databuf=*0x%x, databuf_size=0x%llx, driver_status=*0x%x)", fd, cmd, cmdbuf, cmdbuf_size, databuf, databuf_size, driver_status);

	// cmd == 2 is get device info,
	// databuf, first byte 0 == status ok?
	// byte 1, if < 0 , not ata device
	return CELL_OK;
}

error_code sys_storage_check_region_acl()
{
	sys_storage.todo("sys_storage_check_region_acl()");

	return CELL_OK;
}

error_code sys_storage_set_region_acl()
{
	sys_storage.todo("sys_storage_set_region_acl()");

	return CELL_OK;
}

error_code sys_storage_get_region_offset()
{
	sys_storage.todo("sys_storage_get_region_offset()");

	return CELL_OK;
}

error_code sys_storage_set_emulated_speed()
{
	sys_storage.todo("sys_storage_set_emulated_speed()");

	// todo: only debug kernel has this
	return CELL_ENOSYS;
}
