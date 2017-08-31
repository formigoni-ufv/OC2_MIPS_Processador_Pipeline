module instructionmemory (
	input wire[31:0] addr,
	output reg[31:0] instruction
	);

	reg[31:0] memory[0:63];

	///////////MEMORIA DE INSTRUCOES//////////////////////
	initial begin
		memory[0]  <= 32'b00000001000010011000000000100000;			//add $s0 $t0 $t1 // 8 + 4 = 12
		memory[4]  <= 32'b00000010000010101000100000100010;			//sub $s1 $s0 $t2 // 12 - 10 = 2
		memory[8]  <= 32'b00000010000100011001000000100000;			//add $s2 $s0 $s1 // 12 + 2 = 14
		memory[12] <= 32'b10101100000011000000000000000100;			//sw  $t4 4($0) //Guarda 132 na memoria
		memory[16] <= 32'b10001100000011010000000000000100;			//lw $t5 4($0)	 //Carrega 132 da memoria
		// memory[8]  <= 32'b00010001010010110000000000000001;			//beq $t2 $t3 1   // Avança 1 byte (8 bits)
		// memory[16] <= 32'b00000010000010011000000000100010;			//sub $s0 $s0 $t1 // 12 - 6 = 6  HEX: 6
		// memory[20] <= 32'b10101100000011000000000000000100;			//sw $t4 4($0) //Guarda 77 na memoria
		// memory[24] <= 32'b10001100000011010000000000000100;			//lw $t5 4($0)	 //Carrega 77 da memoria
		// memory[28] <= 32'b00000001101100010110100000100000;			//add $t5 $t5 $s1 // 77 + 10 = 87 HEX: 57
		// memory[32] <= 32'b00010001010010111111111111110111;			//beq $t2 $t3 -9 //Volta ao Inicio
	end

	always @ (addr) begin
		instruction[31:0] = memory[addr];
	end
endmodule
