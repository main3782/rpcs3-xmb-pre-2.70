#include "stdafx.h"
#include "Emu/Cell/PPUModule.h"
#include "Emu/Cell/lv2/sys_rsxaudio.h"

LOG_CHANNEL(sysPrxForUser);

// User space wrappers of the lv2 rsxaudio syscalls. Firmware whose liblv2.sprx has no wrappers for them (2.17) imports them
// from sysPrxForUser instead, for example the audio server module sys_audio.sprx. They used to be unimplemented stubs that
// returned CELL_OK without writing their output, so the module then used a null shared memory address.
// The functions are registered by NID because their names would clash with the syscall implementations.

static error_code sys_rsxaudio_close_connection_hle(u32 handle)
{
	sysPrxForUser.trace("sys_rsxaudio_close_connection(handle=0x%x)", handle);
	return ::sys_rsxaudio_close_connection(handle);
}

static error_code sys_rsxaudio_create_connection_hle(u32 handle)
{
	sysPrxForUser.trace("sys_rsxaudio_create_connection(handle=0x%x)", handle);
	return ::sys_rsxaudio_create_connection(handle);
}

static error_code sys_rsxaudio_finalize_hle(ppu_thread& ppu, u32 handle)
{
	sysPrxForUser.trace("sys_rsxaudio_finalize(handle=0x%x)", handle);
	return ::sys_rsxaudio_finalize(ppu, handle);
}

static error_code sys_rsxaudio_import_shared_memory_hle(ppu_thread& ppu, u32 handle, vm::ptr<u64> addr)
{
	sysPrxForUser.trace("sys_rsxaudio_import_shared_memory(handle=0x%x, addr=*0x%x)", handle, addr);
	return ::sys_rsxaudio_import_shared_memory(ppu, handle, addr);
}

static error_code sys_rsxaudio_initialize_hle(ppu_thread& ppu, vm::ptr<u32> handle)
{
	sysPrxForUser.trace("sys_rsxaudio_initialize(handle=*0x%x)", handle);
	return ::sys_rsxaudio_initialize(ppu, handle);
}

static error_code sys_rsxaudio_prepare_process_hle(u32 handle)
{
	sysPrxForUser.trace("sys_rsxaudio_prepare_process(handle=0x%x)", handle);
	return ::sys_rsxaudio_prepare_process(handle);
}

static error_code sys_rsxaudio_start_process_hle(u32 handle)
{
	sysPrxForUser.trace("sys_rsxaudio_start_process(handle=0x%x)", handle);
	return ::sys_rsxaudio_start_process(handle);
}

static error_code sys_rsxaudio_stop_process_hle(u32 handle)
{
	sysPrxForUser.trace("sys_rsxaudio_stop_process(handle=0x%x)", handle);
	return ::sys_rsxaudio_stop_process(handle);
}

static error_code sys_rsxaudio_unimport_shared_memory_hle(ppu_thread& ppu, u32 handle, vm::ptr<u64> addr)
{
	sysPrxForUser.trace("sys_rsxaudio_unimport_shared_memory(handle=0x%x, addr=*0x%x)", handle, addr);
	return ::sys_rsxaudio_unimport_shared_memory(ppu, handle, addr);
}

void sysPrxForUser_sys_rsxaudio_init()
{
	REG_FNID(sysPrxForUser, 0xB1BD7A61, sys_rsxaudio_close_connection_hle);
	REG_FNID(sysPrxForUser, 0xA9879A33, sys_rsxaudio_create_connection_hle);
	REG_FNID(sysPrxForUser, 0x87C87495, sys_rsxaudio_finalize_hle);
	REG_FNID(sysPrxForUser, 0x2F256B29, sys_rsxaudio_import_shared_memory_hle);
	REG_FNID(sysPrxForUser, 0xC839BB4C, sys_rsxaudio_initialize_hle);
	REG_FNID(sysPrxForUser, 0x7B034C78, sys_rsxaudio_prepare_process_hle);
	REG_FNID(sysPrxForUser, 0x9C9605EE, sys_rsxaudio_start_process_hle);
	REG_FNID(sysPrxForUser, 0xFF34C76C, sys_rsxaudio_stop_process_hle);
	REG_FNID(sysPrxForUser, 0x39E9B068, sys_rsxaudio_unimport_shared_memory_hle);
}
