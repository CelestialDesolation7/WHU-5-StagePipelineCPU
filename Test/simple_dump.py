from capstone import Cs, CS_ARCH_RISCV, CS_MODE_RISCV32


# 读取 coe 文件中的机器码
def load_hex_from_coe(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()

    hex_lines = []
    for line in lines:
        line = line.strip().rstrip(',')
        if line.startswith('memory_initialization_vector'):
            continue
        if line.startswith('memory_initialization_radix'):
            continue
        if line == '':
            continue
        if ';' in line:
            line = line.split(';')[0]
        hex_lines.append(line.lower())

    return hex_lines


# 将十六进制转为字节序列
def hex_to_bytes(hex_lines):
    b = bytearray()
    for h in hex_lines:
        h = h.zfill(8)
        b += bytes.fromhex(h)[::-1]  # RISC-V 是小端存储
    return b


# 反汇编字节码
def disassemble_riscv(byte_code):
    md = Cs(CS_ARCH_RISCV, CS_MODE_RISCV32)
    md.detail = False
    result = []
    for i in md.disasm(byte_code, 0x0):
        result.append(f"{i.address:08x}:\t{i.mnemonic}\t{i.op_str}")
    return result


# 主函数
def main():
    coe_file = "coe_content.coe"
    output_file = "disasm_output.txt"

    hex_lines = load_hex_from_coe(coe_file)
    byte_code = hex_to_bytes(hex_lines)
    asm_lines = disassemble_riscv(byte_code)

    with open(output_file, 'w') as f:
        for line in asm_lines:
            f.write(line + '\n')

    print(f"反汇编完成，结果已保存到 {output_file}")


if __name__ == "__main__":
    main()
