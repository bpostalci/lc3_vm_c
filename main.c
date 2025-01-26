#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/errno.h>

extern unsigned char lc3os_obj[];
extern unsigned int lc3os_obj_len;

/* Definitions */
#define MEMORY_MAX (1 << 16)

/* Memory */
uint16_t memory[MEMORY_MAX];

/* Registers */
enum
{
    R_R0 = 0,
    R_R1,
    R_R2,
    R_R3,
    R_R4,
    R_R5,
    R_R6,
    R_R7,
    R_PC,   /* Program Counter */
    R_COND, /* Condition Flags */
    R_COUNT
};
uint16_t reg[R_COUNT];

/* Opcodes */
enum
{
    OP_BR = 0, /* Branch */
    OP_ADD,    /* Add */
    OP_LD,     /* Load */
    OP_ST,     /* Store */
    OP_JSR,    /* Jump Register */
    OP_AND,    /* Bitwise AND */
    OP_LDR,    /* Load Register */
    OP_STR,    /* Store Register */
    OP_RTI,    /* Unused */
    OP_NOT,    /* Bitwise NOT */
    OP_LDI,    /* Load Indirect */
    OP_STI,    /* Store Indirect */
    OP_JMP,    /* Jump */
    OP_RES,    /* Reserved (Unused) */
    OP_LEA,    /* Load Effective Address */
    OP_TRAP    /* Execute Trap */
};

/* Condition Flags */
enum
{
    FL_POS = 1 << 0, /* Positive */
    FL_ZRO = 1 << 1, /* Zero */
    FL_NEG = 1 << 2  /* Negative */
};

/* Memory-Mapped Registers */
enum
{
    MR_KBSR = 0xFE00, /* Keyboard Status */
    MR_KBDR = 0xFE02, /* Keyboard Data */
    MR_DSR = 0xFE04,
    MR_DDR = 0xFE06,
    MR_MCR = 0xFFFE,
};

/* Trap Codes */
enum
{
    TRAP_GETC = 0x20,
    TRAP_OUT = 0x21,
    TRAP_PUTS = 0x22,
    TRAP_IN = 0x23,
    TRAP_PUTSP = 0x24,
    TRAP_HALT = 0x25
};

/* VM bits */
enum
{
    VM_STATUS_BIT = 1 << 15,
    VM_SIGN_BIT = 1 << 15,
};

struct termios original_tio;

void disable_input_buffering(void)
{
    tcgetattr(STDIN_FILENO, &original_tio);
    struct termios new_tio = original_tio;
    new_tio.c_lflag &= ~ICANON & ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering(void)
{
    tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

void handle_interrupt(int signal)
{
    if (signal != SIGINT)
        return;

    restore_input_buffering();
    printf("\n");
    exit(-2);
}

/* Utility Functions */
uint16_t sign_extend(uint16_t val, int n)
{
    uint16_t m = 1 << (n - 1);
    val &= ((1 << n) - 1);
    return (val ^ m) - m;
}

uint16_t swap16(uint16_t val)
{
    uint16_t high_byte = val << 8;
    uint16_t low_byte = val >> 8;
    return high_byte | low_byte;
}

void update_flags(uint16_t r)
{
    if (reg[r] == 0)
    {
        reg[R_COND] = FL_ZRO;
    }
    else if (reg[r] & VM_SIGN_BIT) /* Check leftmost bit for negative */
    {
        reg[R_COND] = FL_NEG;
    }
    else
    {
        reg[R_COND] = FL_POS;
    }
}

void mem_write(uint16_t addr, uint16_t val)
{
    if (addr == MR_KBSR || addr == MR_KBDR || addr == MR_DSR)
    {
        return;
    }
    else if (addr == MR_DDR)
    {
        putchar(val);
        fflush(stdout);
        return;
    }

    memory[addr] = val;
}

uint16_t mem_read(uint16_t addr)
{
    if (addr == MR_KBSR)
    {
        static fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        return select(1, &readfds, NULL, NULL, &timeout) ? VM_STATUS_BIT : 0;
    }
    else if (addr == MR_KBDR)
    {
        if (mem_read(MR_KBSR))
        {
            return getchar();
        }
        else
        {
            return 0;
        }
    }
    else if (addr == MR_DSR)
    {
        return VM_STATUS_BIT;
    }
    else if (addr == MR_DDR)
    {
        return 0;
    }
    return memory[addr];
}

/* Instruction Implementations */
void execute_add(uint16_t instr)
{
    uint16_t dr = (instr >> 9) & 0x7;
    uint16_t sr1 = (instr >> 6) & 0x7;

    if (instr & (1 << 5))
    {
        uint16_t imm5 = sign_extend(instr, 5);
        reg[dr] = reg[sr1] + imm5;
    }
    else
    {
        uint16_t sr2 = instr & 0x7;
        reg[dr] = reg[sr1] + reg[sr2];
    }

    update_flags(dr);
}

void execute_and(uint16_t instr)
{
    uint16_t dr = (instr >> 9) & 0x7;
    uint16_t sr1 = (instr >> 6) & 0x7;

    if (instr & (1 << 5))
    {
        uint16_t imm5 = sign_extend(instr, 5);
        reg[dr] = reg[sr1] & imm5;
    }
    else
    {
        uint16_t sr2 = instr & 0x7;
        reg[dr] = reg[sr1] & reg[sr2];
    }

    update_flags(dr);
}

void execute_ldi(uint16_t instr)
{
    /* destination register (DR) */
    uint16_t r0 = (instr >> 9) & 0x7;
    /* PCoffset 9*/
    uint16_t pc_offset = sign_extend(instr, 9);
    /* add pc_offset to the current PC, look at that memory location to get the final address */
    reg[r0] = mem_read(mem_read(reg[R_PC] + pc_offset));
    update_flags(r0);
}

void execute_not(uint16_t instr)
{
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t r1 = (instr >> 6) & 0x7;

    reg[r0] = ~reg[r1];
    update_flags(r0);
}

void execute_br(uint16_t instr)
{
    uint16_t pc_offset = sign_extend(instr, 9);
    uint16_t cond_flag = (instr >> 9) & 0x7;
    if (cond_flag & (reg[R_COND] & 0x7))
    {
        reg[R_PC] += pc_offset;
    }
}

void execute_jmp(uint16_t instr)
{
    /* Also handles RET */
    uint16_t r1 = (instr >> 6) & 0x7;
    reg[R_PC] = reg[r1];
}

void execute_jsr(uint16_t instr)
{
    uint16_t original_pc = reg[R_PC];
    if (instr & (1 << 11))
    {
        uint16_t long_pc_offset = sign_extend(instr, 11);
        reg[R_PC] += long_pc_offset; /* JSR */
    }
    else
    {
        uint16_t r1 = (instr >> 6) & 0x7;
        reg[R_PC] = reg[r1]; /* JSRR */
    }

    reg[R_R7] = original_pc;
}

void execute_ld(uint16_t instr)
{
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t pc_offset = sign_extend(instr, 9);
    reg[r0] = mem_read(reg[R_PC] + pc_offset);
    update_flags(r0);
}

void execute_ldr(uint16_t instr)
{
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t r1 = (instr >> 6) & 0x7;
    uint16_t offset = sign_extend(instr, 6);
    reg[r0] = mem_read(reg[r1] + offset);
    update_flags(r0);
}

void execute_lea(uint16_t instr)
{
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t pc_offset = sign_extend(instr, 9);
    reg[r0] = reg[R_PC] + pc_offset;
    update_flags(r0);
}

void execute_st(uint16_t instr)
{
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t pc_offset = sign_extend(instr, 9);
    mem_write(reg[R_PC] + pc_offset, reg[r0]);
}

void execute_sti(uint16_t instr)
{
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t pc_offset = sign_extend(instr, 9);
    mem_write(mem_read(reg[R_PC] + pc_offset), reg[r0]);
}

void execute_str(uint16_t instr)
{
    uint16_t sr = (instr >> 9) & 0x7;
    uint16_t baser = (instr >> 6) & 0x7;
    uint16_t offset6 = sign_extend(instr, 6);
    mem_write(reg[baser] + offset6, reg[sr]);
}

void execute_trap(uint16_t instr)
{
    uint16_t trapvect8 = instr & 0xFF;
    if (trapvect8 == 0x20)
    {
        reg[0] = getchar();
    }
    else
    {
        // fallback to OS implementation of remaining traps
        reg[R_R7] = reg[R_PC];
        reg[R_PC] = mem_read(trapvect8);
    }
}

uint16_t load_data(unsigned const char *data, size_t length)
{
    uint16_t load_addr = swap16(*((uint16_t *)data));
    size_t load_length = (length - sizeof(uint16_t)) / sizeof(uint16_t);

    assert(load_addr + load_length < UINT16_MAX);

    uint16_t *dest = memory + load_addr;
    uint16_t *source = (uint16_t *)(data + sizeof(uint16_t));

    if (dest + load_length >= memory + MEMORY_MAX)
    {
        fprintf(stderr, "data cannot fit in memory, max: %d, error: %s\n", MEMORY_MAX, strerror(errno));
        return 1;
    }

    while (load_length-- > 0)
    {
        *(dest++) = swap16(*(source++));
    }

    reg[R_PC] = load_addr;

    return 0;
}

uint16_t load_file(const char *file)
{
    int fd, ret;
    struct stat statbuf;
    unsigned char *data;

    if ((fd = open(file, O_RDONLY)) < 0 ||
        (ret = fstat(fd, &statbuf)) < 0 ||
        (data = mmap(0, statbuf.st_size, PROT_READ, MAP_SHARED, fd, 0)) == MAP_FAILED)
    {
        return 1;
    }

    uint16_t result = load_data(data, statbuf.st_size);

    munmap(data, statbuf.st_size);
    close(fd);

    return result;
}

uint16_t run_instruction(uint16_t instr)
{
    uint16_t op = instr >> 12;

    switch (op)
    {
    case OP_ADD:
        execute_add(instr);
        break;
    case OP_TRAP:
        execute_trap(instr);
        break;
    case OP_AND:
        execute_and(instr);
        break;
    case OP_NOT:
        execute_not(instr);
        break;
    case OP_BR:
        execute_br(instr);
        break;
    case OP_JMP:
        execute_jmp(instr);
        break;
    case OP_JSR:
        execute_jsr(instr);
        break;
    case OP_LD:
        execute_ld(instr);
        break;
    case OP_LDI:
        execute_ldi(instr);
        break;
    case OP_LDR:
        execute_ldr(instr);
        break;
    case OP_LEA:
        execute_lea(instr);
        break;
    case OP_ST:
        execute_st(instr);
        break;
    case OP_STI:
        execute_sti(instr);
        break;
    case OP_STR:
        execute_str(instr);
        break;
    default:
        return 1;
    }

    return 0;
}

uint16_t run_vm(void)
{
    while (mem_read(MR_MCR) & VM_STATUS_BIT)
    {
        uint16_t res = run_instruction(mem_read(reg[R_PC]++));
        if (res != 0)
        {
            return res;
        }
    }
    return 0;
}

int main(int argc, const char *argv[])
{
    if (argc < 2)
    {
        printf("lc3_vm_c [image-file] ...\n");
        exit(2);
    }

    reg[R_PC] = 0x3000;
    reg[R_COND] = FL_ZRO;
    memory[MR_MCR] = VM_STATUS_BIT;

    if (load_data(lc3os_obj, lc3os_obj_len) != 0)
    {
        fprintf(stderr, "failed to load LC3 OS\n");
        exit(2);
    }

    uint16_t res = load_file(argv[1]);

    if (res == 0)
    {
        // Dump memory after successful load
        FILE *dump_file = fopen("memory_dump.txt", "w");
        if (dump_file != NULL)
        {
            fprintf(dump_file, "Memory dump after loading image:\n");
            for (uint16_t addr = 0; addr < UINT16_MAX; addr++)
            {
                uint16_t value = mem_read(addr);
                if (value != 0)
                {
                    fprintf(dump_file, "0x%04x: 0x%04x\n", addr, value);
                }
            }
            fclose(dump_file);
            printf("Memory dump written to memory_dump.txt\n");
        }
    }
    else
    {
        fprintf(stderr, "%s: file cannot be loaded, error: %s\n", argv[0], strerror(errno));
        exit(2);
    }

    signal(SIGINT, handle_interrupt);
    disable_input_buffering();

    run_vm();

    printf("Exiting VM...\n");

    restore_input_buffering();

    return 0;
}
