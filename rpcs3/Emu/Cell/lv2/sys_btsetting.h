#pragma once

#include "Emu/Memory/vm_ptr.h"
#include "Emu/Cell/ErrorCodes.h"

// SysCalls

error_code sys_btsetting_if(u64 cmd, vm::ptr<void> msg);

struct ppu_thread;

// Syscall 640. Only the shared-memory handshake VSH 1.xx's libusbbtaudio proxy asserts on is implemented.
error_code sys_usbbtaudio_initialize(ppu_thread& ppu);
