#include "stdafx.h"
#include "Emu/Cell/ErrorCodes.h"
#include "Emu/Cell/PPUThread.h"

#include "sys_console.h"


LOG_CHANNEL(sys_console);

error_code sys_console_write(vm::cptr<char> buf, u32 len)
{
	// Diagnostic only (0.85 VSH bring-up): the call is still a stub that returns CELL_OK,
	// but the text VSH writes is now logged so it can be read.
	std::string msg;

	if (len > 0u && len <= 0x10000u && vm::check_addr(buf.addr(), vm::page_readable, len))
	{
		msg.resize(len);

		if (!vm::try_access(buf.addr(), msg.data(), len, false))
		{
			msg.clear();
		}
	}

	// Keep the log line readable: printable ASCII only, newlines shown as "|"
	std::string clean;
	clean.reserve(msg.size());

	for (const char c : msg)
	{
		if (c == '\n' || c == '\r')
		{
			clean += '|';
		}
		else if (c >= 0x20 && c < 0x7f)
		{
			clean += c;
		}
		else if (c != '\0')
		{
			clean += '?';
		}
	}

	if (clean.size() > 512u)
	{
		clean.resize(512u);
	}

	sys_console.warning("sys_console_write(buf=*0x%x, len=0x%x): \"%s\"", buf, len, clean);

	return CELL_OK;
}
