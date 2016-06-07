#include "stdafx.h"
#include "Utilities/Config.h"
#include "Emu/Memory/Memory.h"
#include "Emu/System.h"
#include "Emu/IdManager.h"
#include "PPUThread.h"
#include "PPUInterpreter.h"
#include "PPUTranslator.h"
#include "PPUAnalyser.h"
#include "PPUModule.h"

enum class ppu_decoder_type
{
	precise,
	fast,
	llvm,
};

cfg::map_entry<ppu_decoder_type> g_cfg_ppu_decoder(cfg::root.core, "PPU Decoder", 1,
{
	{ "Interpreter (precise)", ppu_decoder_type::precise },
	{ "Interpreter (fast)", ppu_decoder_type::fast },
	{ "Recompiler (LLVM)", ppu_decoder_type::llvm },
});

const ppu_decoder<ppu_interpreter_precise> s_ppu_interpreter_precise;
const ppu_decoder<ppu_interpreter_fast> s_ppu_interpreter_fast;

struct ppu_addr_hash
{
	u32 operator()(u32 value) const
	{
		return value / sizeof(32);
	}
};

static std::unordered_map<u32, void(*)(), ppu_addr_hash> s_ppu_compiled;



std::string PPUThread::get_name() const
{
	return fmt::format("PPU[0x%x] Thread (%s)", id, name);
}

std::string PPUThread::dump() const
{
	std::string ret = "Registers:\n=========\n";

	for (uint i = 0; i<32; ++i) ret += fmt::format("GPR[%d] = 0x%llx\n", i, GPR[i]);
	for (uint i = 0; i<32; ++i) ret += fmt::format("FPR[%d] = %.6G\n", i, FPR[i]);
	for (uint i = 0; i<32; ++i) ret += fmt::format("VR[%d] = 0x%s [%s]\n", i, VR[i].to_hex().c_str(), VR[i].to_xyzw().c_str());
	ret += fmt::format("CR = 0x%08x\n", GetCR());
	ret += fmt::format("LR = 0x%llx\n", LR);
	ret += fmt::format("CTR = 0x%llx\n", CTR);
	ret += fmt::format("XER = [CA=%u | OV=%u | SO=%u | CNT=%u]\n", u32{ CA }, u32{ OV }, u32{ SO }, u32{ XCNT });
	//ret += fmt::format("FPSCR = 0x%x "
	//	"[RN=%d | NI=%d | XE=%d | ZE=%d | UE=%d | OE=%d | VE=%d | "
	//	"VXCVI=%d | VXSQRT=%d | VXSOFT=%d | FPRF=%d | "
	//	"FI=%d | FR=%d | VXVC=%d | VXIMZ=%d | "
	//	"VXZDZ=%d | VXIDI=%d | VXISI=%d | VXSNAN=%d | "
	//	"XX=%d | ZX=%d | UX=%d | OX=%d | VX=%d | FEX=%d | FX=%d]\n",
	//	FPSCR.FPSCR,
	//	u32{ FPSCR.RN },
	//	u32{ FPSCR.NI }, u32{ FPSCR.XE }, u32{ FPSCR.ZE }, u32{ FPSCR.UE }, u32{ FPSCR.OE }, u32{ FPSCR.VE },
	//	u32{ FPSCR.VXCVI }, u32{ FPSCR.VXSQRT }, u32{ FPSCR.VXSOFT }, u32{ FPSCR.FPRF },
	//	u32{ FPSCR.FI }, u32{ FPSCR.FR }, u32{ FPSCR.VXVC }, u32{ FPSCR.VXIMZ },
	//	u32{ FPSCR.VXZDZ }, u32{ FPSCR.VXIDI }, u32{ FPSCR.VXISI }, u32{ FPSCR.VXSNAN },
	//	u32{ FPSCR.XX }, u32{ FPSCR.ZX }, u32{ FPSCR.UX }, u32{ FPSCR.OX }, u32{ FPSCR.VX }, u32{ FPSCR.FEX }, u32{ FPSCR.FX });

	return ret;
}

void PPUThread::cpu_init()
{
	if (!stack_addr)
	{
		if (!stack_size)
		{
			throw EXCEPTION("Invalid stack size");
		}

		stack_addr = vm::alloc(stack_size, vm::stack);

		if (!stack_addr)
		{
			throw EXCEPTION("Out of stack memory");
		}
	}

	GPR[1] = align(stack_addr + stack_size, 0x200) - 0x200;
}

extern thread_local std::string(*g_tls_log_prefix)();

void PPUThread::cpu_task()
{
	//SetHostRoundingMode(FPSCR_RN_NEAR);

	if (custom_task)
	{
		if (check_status()) return;

		return custom_task(*this);
	}

	if (g_cfg_ppu_decoder.get() == ppu_decoder_type::llvm)
	{
		const auto found = s_ppu_compiled.find(pc);

		if (found != s_ppu_compiled.end())
		{
			return found->second();
		}
	}

	g_tls_log_prefix = []
	{
		const auto cpu = static_cast<PPUThread*>(get_current_cpu_thread());

		return fmt::format("%s [0x%08x]", cpu->get_name(), cpu->pc);
	};

	const auto base = vm::_ptr<const u8>(0);

	// Select opcode table
	const auto& table = *(
		g_cfg_ppu_decoder.get() == ppu_decoder_type::precise ? &s_ppu_interpreter_precise.get_table() :
		g_cfg_ppu_decoder.get() == ppu_decoder_type::fast ? &s_ppu_interpreter_fast.get_table() :
		throw std::logic_error("Invalid PPU decoder"));

	v128 _op;
	decltype(&ppu_interpreter::UNK) func0, func1, func2, func3;

	while (true)
	{
		if (UNLIKELY(state.load()))
		{
			if (check_status()) return;
		}

		// Reinitialize
		{
			const auto _ops = _mm_shuffle_epi8(_mm_lddqu_si128(reinterpret_cast<const __m128i*>(base + pc)), _mm_set_epi8(12, 13, 14, 15, 8, 9, 10, 11, 4, 5, 6, 7, 0, 1, 2, 3));
			_op.vi = _ops;
			const v128 _i = v128::fromV(_mm_and_si128(_mm_or_si128(_mm_slli_epi32(_op.vi, 6), _mm_srli_epi32(_op.vi, 26)), _mm_set1_epi32(0x1ffff)));
			func0 = table[_i._u32[0]];
			func1 = table[_i._u32[1]];
			func2 = table[_i._u32[2]];
			func3 = table[_i._u32[3]];
		}

		while (LIKELY(func0(*this, { _op._u32[0] })))
		{
			if (pc += 4, LIKELY(func1(*this, { _op._u32[1] })))
			{
				if (pc += 4, LIKELY(func2(*this, { _op._u32[2] })))
				{
					pc += 4;
					func0 = func3;

					const auto _ops = _mm_shuffle_epi8(_mm_lddqu_si128(reinterpret_cast<const __m128i*>(base + pc + 4)), _mm_set_epi8(12, 13, 14, 15, 8, 9, 10, 11, 4, 5, 6, 7, 0, 1, 2, 3));
					_op.vi = _mm_alignr_epi8(_ops, _op.vi, 12);
					const v128 _i = v128::fromV(_mm_and_si128(_mm_or_si128(_mm_slli_epi32(_op.vi, 6), _mm_srli_epi32(_op.vi, 26)), _mm_set1_epi32(0x1ffff)));
					func1 = table[_i._u32[1]];
					func2 = table[_i._u32[2]];
					func3 = table[_i._u32[3]];

					if (UNLIKELY(state.load()))
					{
						break;
					}
					continue;
				}
				break;
			}
			break;
		}
	}
}

constexpr auto stop_state = make_bitset(cpu_state::stop, cpu_state::exit, cpu_state::suspend);

atomic_t<u32> g_ppu_core[2]{};

bool PPUThread::handle_interrupt()
{
	// Reschedule and wake up a new thread, possibly this one as well.
	return false;

	// Check virtual core allocation
	if (g_ppu_core[0] != id && g_ppu_core[1] != id)
	{
		auto cpu0 = idm::get<PPUThread>(g_ppu_core[0]);
		auto cpu1 = idm::get<PPUThread>(g_ppu_core[1]);

		if (cpu0 && cpu1)
		{
			if (cpu1->prio > cpu0->prio)
			{
				cpu0 = std::move(cpu1);
			}

			// Preempt thread with the lowest priority
			if (prio < cpu0->prio)
			{
				cpu0->state += cpu_state::interrupt;
			}
		}
		else
		{
			// Try to obtain a virtual core in optimistic way
			if (g_ppu_core[0].compare_and_swap_test(0, id) || g_ppu_core[1].compare_and_swap_test(0, id))
			{
				state -= cpu_state::interrupt;
				return true;
			}
		}

		return false;
	}

	// Select appropriate thread
	u32 top_prio = -1;
	u32 selected = -1;

	idm::select<PPUThread>([&](u32 id, PPUThread& ppu)
	{
		// Exclude suspended and low-priority threads
		if (!ppu.state.test(stop_state) && ppu.prio < top_prio /*&& (!ppu.is_sleep() || ppu.state & cpu_state::signal)*/)
		{
			top_prio = ppu.prio;
			selected = id;
		}
	});

	// If current thread selected
	if (selected == id)
	{
		state -= cpu_state::interrupt;
		VERIFY(g_ppu_core[0] == id || g_ppu_core[1] == id);
		return true;
	}

	// If another thread selected
	const auto thread = idm::get<PPUThread>(selected);

	// Lend virtual core to another thread
	if (thread && thread->state.test_and_reset(cpu_state::interrupt))
	{
		g_ppu_core[0].compare_and_swap(id, thread->id);
		g_ppu_core[1].compare_and_swap(id, thread->id);
		(*thread)->lock_notify();
	}
	else
	{
		g_ppu_core[0].compare_and_swap(id, 0);
		g_ppu_core[1].compare_and_swap(id, 0);
	}

	return false;
}

PPUThread::~PPUThread()
{
	if (stack_addr)
	{
		vm::dealloc_verbose_nothrow(stack_addr, vm::stack);
	}
}

PPUThread::PPUThread(const std::string& name)
	: cpu_thread(cpu_type::ppu, name)
{
}

be_t<u64>* PPUThread::get_stack_arg(s32 i, u64 align)
{
	if (align != 1 && align != 2 && align != 4 && align != 8 && align != 16) throw fmt::exception("Unsupported alignment: 0x%llx" HERE, align);
	return vm::_ptr<u64>(vm::cast((GPR[1] + 0x30 + 0x8 * (i - 1)) & (0 - align), HERE));
}

void PPUThread::fast_call(u32 addr, u32 rtoc)
{
	auto old_PC = pc;
	auto old_stack = GPR[1];
	auto old_rtoc = GPR[2];
	auto old_LR = LR;
	auto old_task = std::move(custom_task);

	pc = addr;
	GPR[2] = rtoc;
	LR = Emu.GetCPUThreadStop();
	custom_task = nullptr;

	try
	{
		cpu_task();
	}
	catch (cpu_state _s)
	{
		state += _s;
		if (_s != cpu_state::ret) throw;
	}

	state -= cpu_state::ret;

	pc = old_PC;

	if (GPR[1] != old_stack) // GPR[1] shouldn't change
	{
		throw EXCEPTION("Stack inconsistency (addr=0x%x, rtoc=0x%x, SP=0x%llx, old=0x%llx)", addr, rtoc, GPR[1], old_stack);
	}

	GPR[2] = old_rtoc;
	LR = old_LR;
	custom_task = std::move(old_task);

	//if (custom_task)
	//{
	//	state += cpu_state::interrupt;
	//	handle_interrupt();
	//}
}

#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/Host.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/Debug.h"
#include "llvm/CodeGen/CommandFlags.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Verifier.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/Analysis/Passes.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/Analysis/MemoryDependenceAnalysis.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/IPO.h"
#include "llvm/PassManager.h"

#include "llvm/ExecutionEngine/ExecutionEngine.h"
#include "llvm/ExecutionEngine/MCJIT.h"
#include "llvm/ExecutionEngine/SectionMemoryManager.h"
#include "llvm/ExecutionEngine/JITEventListener.h"
#include "llvm/Object/ObjectFile.h"
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#ifdef _WIN32
#include <Windows.h>
#endif

const ppu_decoder<ppu_itype> s_ppu_itype;

extern u64 get_timebased_time();
extern void ppu_execute_syscall(PPUThread& ppu, u64 code);
extern void ppu_execute_function(PPUThread& ppu, u32 index);

extern __m128 sse_exp2_ps(__m128 A);
extern __m128 sse_log2_ps(__m128 A);
extern __m128i sse_altivec_vperm(__m128i A, __m128i B, __m128i C);
extern __m128i sse_altivec_lvsl(u64 addr);
extern __m128i sse_altivec_lvsr(u64 addr);
extern __m128i sse_cellbe_lvlx(u64 addr);
extern __m128i sse_cellbe_lvrx(u64 addr);
extern void sse_cellbe_stvlx(u64 addr, __m128i a);
extern void sse_cellbe_stvrx(u64 addr, __m128i a);

struct MemoryManager : SectionMemoryManager
{
	static PPUThread* context(u64 addr)
	{
		//LOG_NOTICE(PPU, "Trace: 0x%llx", addr);
		return static_cast<PPUThread*>(get_current_cpu_thread());
	}

	[[noreturn]] static void trap(u64 addr)
	{
		LOG_FATAL(PPU, "Trap! (0x%llx)", addr);
		throw fmt::exception("Trap! (0x%llx)", addr);
	}

	static void hack(u32 index)
	{
		ppu_execute_function(static_cast<PPUThread&>(*get_current_cpu_thread()), index);
	}

	static void syscall(u64 code)
	{
		ppu_execute_syscall(static_cast<PPUThread&>(*get_current_cpu_thread()), code);
	}

	static u32 tbl()
	{
		return (u32)get_timebased_time();
	}

	static void call(u32 addr)
	{
		const auto found = s_ppu_compiled.find(addr);

		if (found != s_ppu_compiled.end())
		{
			return found->second();
		}

		const auto op = vm::read32(addr).value();
		const auto itype = s_ppu_itype.decode(op);

		if (itype == ppu_itype::HACK && vm::read32(addr + 4) == ppu_instructions::BLR())
		{
			return hack(op & 0x3ffffff);
		}

		trap(addr);
	}

	static __m128 sse_rcp_ps(__m128 A)
	{
		return _mm_rcp_ps(A);
	}

	static __m128 sse_rsqrt_ps(__m128 A)
	{
		return _mm_rsqrt_ps(A);
	}

	static float sse_rcp_ss(float A)
	{
		_mm_store_ss(&A, _mm_rcp_ss(_mm_load_ss(&A)));
		return A;
	}

	static float sse_rsqrt_ss(float A)
	{
		_mm_store_ss(&A, _mm_rsqrt_ss(_mm_load_ss(&A)));
		return A;
	}

	static u32 lwarx(u32 addr)
	{
		be_t<u32> reg_value;
		vm::reservation_acquire(&reg_value, addr, sizeof(reg_value));
		return reg_value;
	}

	static u64 ldarx(u32 addr)
	{
		be_t<u64> reg_value;
		vm::reservation_acquire(&reg_value, addr, sizeof(reg_value));
		return reg_value;
	}

	static bool stwcx(u32 addr, u32 reg_value)
	{
		const be_t<u32> data = reg_value;
		return vm::reservation_update(addr, &data, sizeof(data));
	}

	static bool stdcx(u32 addr, u64 reg_value)
	{
		const be_t<u64> data = reg_value;
		return vm::reservation_update(addr, &data, sizeof(data));
	}

	static __m128i fake()
	{
		return _mm_setzero_si128(); // TODO: remove this
	}

	std::unordered_map<std::string, u64> table
	{
		{ "__context", (u64)&context },
		{ "__trap", (u64)&trap },
		{ "__hack", (u64)&hack },
		{ "__memory", (u64)&vm::g_base_addr },
		{ "__syscall", (u64)&syscall },
		{ "__get_tbl", (u64)&tbl },
		{ "__call", (u64)&call },
		{ "__lwarx", (u64)&lwarx },
		{ "__ldarx", (u64)&ldarx },
		{ "__stwcx", (u64)&stwcx },
		{ "__stdcx", (u64)&stdcx },
		{ "__vexptefp", (u64)&sse_exp2_ps },
		{ "__vlogefp", (u64)&sse_log2_ps },
		{ "__vperm", (u64)&sse_altivec_vperm },
		{ "__vrefp", (u64)&sse_rcp_ps },
		{ "__vrsqrtefp", (u64)&sse_rsqrt_ps },
		{ "__vsum4sbs", (u64)&fake },
		{ "__vsum4shs", (u64)&fake },
		{ "__vmsumshs", (u64)&fake },
		{ "__vmsumuhs", (u64)&fake },
		{ "__lvsl", (u64)&sse_altivec_lvsl },
		{ "__lvsr", (u64)&sse_altivec_lvsr },
		{ "__lvlx", (u64)&sse_cellbe_lvlx },
		{ "__lvrx", (u64)&sse_cellbe_lvrx },
		{ "__stvlx", (u64)&sse_cellbe_stvlx },
		{ "__stvrx", (u64)&sse_cellbe_stvrx },
		{ "__fre", (u64)&sse_rcp_ss },
		{ "__frsqrte", (u64)&sse_rsqrt_ss },
	};

	virtual u64 getSymbolAddress(const std::string& name) override
	{
		if (u64 addr = getSymbolAddressInProcess(name))
		{
			return addr;
		}

		const auto found = table.find(name);

		if (found != table.end())
		{
			return found->second;
		}

		LOG_FATAL(PPU, "LLVM: Linking failed at %s", name);
		return (u64)trap;
	}

	/* Example:
	0x00000000062C0000  14 00 00 00 00 00 00 00 03 7a 52 00 01 78 10 01  .........zR..x..
	0x00000000062C0010  1c 0c 07 08 90 01 00 00 24 00 00 00 1c 00 00 00  ....・...$.......
	0x00000000062C0020  e0 ff 5a 0f 00 00 00 00 5c 00 00 00 00 00 00 00  ・.Z.....\.......
	0x00000000062C0030  00 41 0e 10 44 0e 30 84 02 00 00 00 00 00 00 00  .A..D.0・........
	0x00000000062C0040  2c 00 00 00 44 00 00 00 18 00 5b 0f 00 00 00 00  ,...D.....[.....
	0x00000000062C0050  2a 01 00 00 00 00 00 00 00 42 0e 10 41 0e 18 41  *........B..A..A
	0x00000000062C0060  0e 20 41 0e 28 44 0e 50 83 05 85 04 84 03 8e 02  . A.(D.P・.・.・.・.
	0x00000000062C0070  1c 00 00 00 74 00 00 00 18 01 5b 0f 00 00 00 00  ....t.....[.....
	0x00000000062C0080  23 00 00 00 00 00 00 00 00 44 0e 30 00 00 00 00  #........D.0....
	0x00000000062C0090  1c 00 00 00 94 00 00 00 28 01 5b 0f 00 00 00 00  ....・...(.[.....
	0x00000000062C00A0  1a 00 00 00 00 00 00 00 00 44 0e 30 00 00 00 00  .........D.0....
	0x00000000062C00B0  1c 00 00 00 b4 00 00 00 28 01 5b 0f 00 00 00 00  ....ｴ...(.[.....
	0x00000000062C00C0  1f 00 00 00 00 00 00 00 00 44 0e 30 00 00 00 00  .........D.0....
	0x00000000062C00D0  24 00 00 00 d4 00 00 00 28 01 5b 0f 00 00 00 00  $...ﾔ...(.[.....
	0x00000000062C00E0  74 00 00 00 00 00 00 00 00 41 0e 10 44 0e 30 84  t........A..D.0・
	0x00000000062C00F0  02 00 00 00 00 00 00 00 1c 00 00 00 fc 00 00 00  ............・...
	0x00000000062C0100  80 01 5b 0f 00 00 00 00 23 00 00 00 00 00 00 00  ..[.....#.......
	0x00000000062C0110  00 44 0e 30 00 00 00 00 24 00 00 00 1c 01 00 00  .D.0....$.......
	0x00000000062C0120  90 01 5b 0f 00 00 00 00 a3 00 00 00 00 00 00 00  ・.[.....｣.......
	0x00000000062C0130  00 41 0e 10 41 0e 18 44 0e 40 85 03 84 02 00 00  .A..A..D.@・.・...
	0x00000000062C0140  1c 00 00 00 44 01 00 00 18 02 5b 0f 00 00 00 00  ....D.....[.....
	0x00000000062C0150  11 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
	0x00000000062C0160  24 00 00 00 64 01 00 00 18 02 5b 0f 00 00 00 00  $...d.....[.....
	0x00000000062C0170  64 00 00 00 00 00 00 00 00 41 0e 10 44 0e 30 84  d........A..D.0・
	0x00000000062C0180  02 00 00 00 00 00 00 00 24 00 00 00 8c 01 00 00  ........$...・...
	0x00000000062C0190  60 02 5b 0f 00 00 00 00 58 00 00 00 00 00 00 00  `.[.....X.......
	0x00000000062C01A0  00 41 0e 10 44 0e 30 84 02 00 00 00 00 00 00 00  .A..D.0・........
	0x00000000062C01B0  1c 00 00 00 b4 01 00 00 98 02 5b 0f 00 00 00 00  ....ｴ...・.[.....
	0x00000000062C01C0  5c 00 00 00 00 00 00 00 00 44 0e 30 00 00 00 00  \........D.0....
	0x00000000062C01D0  24 00 00 00 d4 01 00 00 d8 02 5b 0f 00 00 00 00  $...ﾔ...ﾘ.[.....
	0x00000000062C01E0  ca 00 00 00 00 00 00 00 00 41 0e 10 41 0e 18 44  ﾊ........A..A..D
	0x00000000062C01F0  0e 40 85 03 84 02 00 00 1c 00 00 00 fc 01 00 00  .@・.・.......・...
	0x00000000062C0200  80 03 5b 0f 00 00 00 00 46 00 00 00 00 00 00 00  ..[.....F.......
	0x00000000062C0210  00 44 0e 30 00 00 00 00 1c 00 00 00 1c 02 00 00  .D.0............
	0x00000000062C0220  b0 03 5b 0f 00 00 00 00 46 00 00 00 00 00 00 00  ｰ.[.....F.......
	0x00000000062C0230  00 44 0e 30 00 00 00 00 24 00 00 00 3c 02 00 00  .D.0....$...<...
	0x00000000062C0240  e0 03 5b 0f 00 00 00 00 83 00 00 00 00 00 00 00  ・.[.....・.......
	0x00000000062C0250  00 41 0e 10 44 0e 30 84 02 00 00 00 00 00 00 00  .A..D.0・........
	0x00000000062C0260  24 00 00 00 64 02 00 00 48 04 5b 0f 00 00 00 00  $...d...H.[.....
	*/

	//virtual void registerEHFrames(u8* addr, u64 load_addr, std::size_t size) override
	//{
	//	u64 min_addr = (u64)addr;

	//	std::vector<RUNTIME_FUNCTION> result;

	//	for (u8* ptr = addr; ptr < addr + size; ptr += 4 + *(u32*)ptr)
	//	{
	//		if (const u32 offset = *(u32*)(ptr + 4))
	//		{
	//			u8* const func = *(u64*)(ptr + 8) + offset + 4 + addr;
	//			u64 const size = *(u64*)(ptr + 16);

	//			min_addr = std::min<u64>(min_addr, (u64)func);

	//			RUNTIME_FUNCTION rtf;
	//			rtf.BeginAddress = (u32)(func - addr);
	//			rtf.EndAddress = (u32)(func - addr + size);
	//			rtf.UnwindData = (u32)(ptr - addr + 24);
	//			result.emplace_back(rtf);
	//		}
	//	}

	//	if (min_addr == load_addr && RtlAddFunctionTable(result.data(), result.size(), load_addr))
	//	{
	//		LOG_SUCCESS(PPU, "LLVM: registerEHFrames(%p, 0x%llx, 0x%llx) (%zu)", addr, load_addr, size, result.size());
	//	}
	//}

	//virtual void deregisterEHFrames(u8* addr, u64 load_addr, std::size_t size) override
	//{
	//}
};

struct Listener : JITEventListener
{
	virtual void NotifyObjectEmitted(const object::ObjectFile& obj, const RuntimeDyld::LoadedObjectInfo&) override
	{
		const StringRef elf = obj.getData();
		fs::file(fs::get_config_dir() + "LLVM.elf", fs::rewrite)
			.write(elf.data(), elf.size());
	}
};

static Listener s_listener;

extern void ppu_initialize(const std::string& name, const std::vector<std::pair<u32, u32>>& funcs, u32 rtoc)
{
	if (g_cfg_ppu_decoder.get() != ppu_decoder_type::llvm || funcs.empty()) return;

	using namespace llvm;

	InitializeAllTargets();
	InitializeAllTargetMCs();
	InitializeAllAsmPrinters();
	LLVMLinkInMCJIT();

	// Initialize LLVM context
	auto& context = getGlobalContext();

	// Create LLVM module
	std::unique_ptr<Module> module = std::make_unique<Module>(name, context);

	// Initialize target
	module->setTargetTriple(Triple::normalize(sys::getProcessTriple() + " -elf"));
	
	// Initialize translator
	std::unique_ptr<PPUTranslator> translator = std::make_unique<PPUTranslator>(getGlobalContext(), module.get(), 0, rtoc);

	// Initialize function list
	for (const auto& info : funcs)
	{
		translator->AddFunction(info.first, cast<Function>(module->getOrInsertFunction(fmt::format("__sub_%x", info.first), FunctionType::get(Type::getVoidTy(context), false))));
	}

	// Translate functions
	for (const auto& info : funcs)
	{
		translator->TranslateToIR(info.first, info.first + info.second, vm::_ptr<u32>(info.first));
	}
	
	legacy::PassManager pm;

	// Basic optimizations
	pm.add(new DataLayoutPass());
	pm.add(createCFGSimplificationPass());
	pm.add(createPromoteMemoryToRegisterPass());
	//pm.add(createNoAAPass());
	//pm.add(createBasicAliasAnalysisPass());
	//pm.add(createNoTargetTransformInfoPass());
	pm.add(createEarlyCSEPass());
	pm.add(createTailCallEliminationPass());
	pm.add(createReassociatePass());
	pm.add(createInstructionCombiningPass());
	pm.add(new DominatorTreeWrapperPass());
	pm.add(createInstructionCombiningPass());
	pm.add(new MemoryDependenceAnalysis());
	pm.add(createDeadStoreEliminationPass());
	pm.add(new LoopInfo());
	pm.add(new ScalarEvolution());

	pm.add(createSCCPPass());
	//pm.add(new SyscallAnalysisPass()); // Requires constant propagation
	pm.add(createInstructionCombiningPass());
	pm.add(createAggressiveDCEPass());
	pm.add(createCFGSimplificationPass());

	// Remove unused functions, structs, global variables, etc
	pm.add(createStripDeadPrototypesPass());
	pm.run(*module);

	std::string result;
	raw_string_ostream out(result);

	out << *module; // print IR
	fs::file(fs::get_config_dir() + "LLVM.log", fs::rewrite)
		.write(out.str());

	result.clear();

	if (verifyModule(*module, &out))
	{
		out.flush();
		LOG_ERROR(PPU, "{%s} LLVM: Translation failed:\n%s", name, result);
		return;
	}

	LOG_SUCCESS(PPU, "LLVM: %zu functions generated", funcs.size());

	Module* module_ptr = module.get();

	std::shared_ptr<ExecutionEngine> engine(EngineBuilder(std::move(module))
		.setErrorStr(&result)
		.setMCJITMemoryManager(std::make_unique<MemoryManager>())
		.setOptLevel(llvm::CodeGenOpt::Aggressive)
		.setMCPU(sys::getHostCPUName())
		.create());

	if (!engine)
	{
		LOG_FATAL(PPU, "LLVM: Failed to create ExecutionEngine: %s", result);
		return;
	}

	engine->RegisterJITEventListener(&s_listener);
	engine->finalizeObject();

	s_ppu_compiled.clear();

	// Get function addresses
	for (const auto& info : funcs)
	{
		const std::uintptr_t link = engine->getFunctionAddress(fmt::format("__sub_%x", info.first));
		s_ppu_compiled.emplace(info.first, (void(*)())link);

		LOG_NOTICE(PPU, "** Function __sub_%x -> 0x%llx (addr=0x%x, size=0x%x)", info.first, link, info.first, info.second);
	}

	// Delete IR to decrease memory consumption
	for (auto& func : module_ptr->functions())
	{
		func.deleteBody();
	}

	fxm::import<ExecutionEngine>(WRAP_EXPR(engine));

	LOG_SUCCESS(PPU, "LLVM: Compilation finished (%s)", sys::getHostCPUName().data());
}
