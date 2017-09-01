`include "processor.v"

module testBench();
	reg clk;
	reg resetManual;
	reg pcInManual;
	
	//Testing
	//Abrindo arquivo de saida
	integer f;

    wire[1:0] CSignal_ForwardingMUX_ALUi0;
    wire[1:0] CSignal_ForwardingMUX_ALUi1;
    wire[31:0] PCOutput;
    wire[31:0] ALUPCPlus4Output;
    wire[31:0] instruction;
    wire PCWrite;
    wire IF_ID_Write;
    wire MUX_ID_EX_Write;
    wire[31:0] PIPE_IFID_ALUPCPlus4Output;
    wire[31:0] PIPE_IFID_Instruction;
    wire[31:0] readData1;
    wire[31:0] readData2;
    wire[31:0] signExtendOutput;
    wire CSignal_RegDst;
    wire CSignal_ALUSrc;
    wire CSignal_MemtoReg;
    wire CSignal_RegWrite;
    wire CSignal_MemRead;
    wire CSignal_MemWrite;
    wire CSignal_Branch;
    wire[1:0] CSignal_ALUOp;
    wire[31:0] PIPE_IDEX_OUT_ALUPCPlus4Output;
    wire[31:0] PIPE_IDEX_OUT_ReadData1;
    wire[31:0] PIPE_IDEX_OUT_ReadData2;
    wire[31:0] PIPE_IDEX_OUT_SignExt;
    wire[4:0] PIPE_IDEX_OUT_RS;
    wire[4:0] PIPE_IDEX_OUT_RT;
    wire[4:0] PIPE_IDEX_OUT_RD;
    wire[31:0] sllOutput;
    wire[31:0] branchALUOutput;
    wire[31:0] ALUSrcOutput;
    wire[31:0] forwardingMUXALUi0;
    wire[31:0] forwardingMUXALUi1;
    wire[31:0] mainALUOutput;
    wire zero;
    wire[3:0] ALUControlOutput;
    wire[4:0] regDstOutput;
    wire PIPE_IDEX_OUT_CSignal_EX_RegDst;
    wire PIPE_IDEX_OUT_CSignal_EX_ALUSrc;
    wire PIPE_IDEX_OUT_CSignal_WB_MemtoReg;
    wire PIPE_IDEX_OUT_CSignal_WB_RegWrite;
    wire PIPE_IDEX_OUT_CSignal_MEM_MRead;
    wire PIPE_IDEX_OUT_CSignal_MEM_MWrite;
    wire PIPE_IDEX_OUT_CSignal_MEM_Branch;
    wire[1:0] PIPE_IDEX_OUT_CSignal_EX_ALUOp;
    wire PIPE_EXMEM_OUT_CSignal_WB_MemtoReg;
    wire PIPE_EXMEM_OUT_CSignal_WB_RegWrite;
    wire PIPE_EXMEM_OUT_CSignal_MEM_MRead;
    wire PIPE_EXMEM_OUT_CSignal_MEM_MWrite;
    wire PIPE_EXMEM_OUT_CSignal_MEM_Branch;
    wire[31:0] PIPE_EXMEM_OUT_BranchALUOutput;
    wire PIPE_EXMEM_OUT_Zero;
    wire[31:0] PIPE_EXMEM_OUT_MainALUOutput;
    wire[31:0] PIPE_EXMEM_OUT_ReadData2;
    wire[4:0] PIPE_EXMEM_OUT_RegDstOutput;
    wire[31:0] dataMemoryOutput;
    wire branchGateOutput;
    wire PIPE_MEMWB_OUT_CSignal_MemtoReg;
    wire PIPE_MEMWB_OUT_CSignal_RegWrite;
    wire[31:0] PIPE_MEMWB_DataMemoryOutput;
    wire[31:0] PIPE_MEMWB_MainALUOutput;
    wire[4:0] PIPE_MEMWB_RegDstOutput;
    wire[31:0] memtoRegOutput;
    processor MIPS(
        .CSignal_ForwardingMUX_ALUi0(CSignal_ForwardingMUX_ALUi0),
        .CSignal_ForwardingMUX_ALUi1(CSignal_ForwardingMUX_ALUi1),
        .PCOutput(PCOutput),
        .ALUPCPlus4Output(ALUPCPlus4Output),
        .instruction(instruction),
        .PCWrite(PCWrite),
        .IF_ID_Write(IF_ID_Write),
        .MUX_ID_EX_Write(MUX_ID_EX_Write),
        .PIPE_IFID_ALUPCPlus4Output(PIPE_IFID_ALUPCPlus4Output),
        .PIPE_IFID_Instruction(PIPE_IFID_Instruction),
        .readData1(readData1),
        .readData2(readData2),
        .signExtendOutput(signExtendOutput),
        .CSignal_RegDst(CSignal_RegDst),
        .CSignal_ALUSrc(CSignal_ALUSrc),
        .CSignal_MemtoReg(CSignal_MemtoReg),
        .CSignal_RegWrite(CSignal_RegWrite),
        .CSignal_MemRead(CSignal_MemRead),
        .CSignal_MemWrite(CSignal_MemWrite),
        .CSignal_Branch(CSignal_Branch),
        .CSignal_ALUOp(CSignal_ALUOp),
        .PIPE_IDEX_OUT_ALUPCPlus4Output(PIPE_IDEX_OUT_ALUPCPlus4Output),
        .PIPE_IDEX_OUT_ReadData1(PIPE_IDEX_OUT_ReadData1),
        .PIPE_IDEX_OUT_ReadData2(PIPE_IDEX_OUT_ReadData2),
        .PIPE_IDEX_OUT_SignExt(PIPE_IDEX_OUT_SignExt),
        .PIPE_IDEX_OUT_RS(PIPE_IDEX_OUT_RS),
        .PIPE_IDEX_OUT_RT(PIPE_IDEX_OUT_RT),
        .PIPE_IDEX_OUT_RD(PIPE_IDEX_OUT_RD),
        .sllOutput(sllOutput),
        .branchALUOutput(branchALUOutput),
        .ALUSrcOutput(ALUSrcOutput),
        .forwardingMUXALUi0(forwardingMUXALUi0),
        .forwardingMUXALUi1(forwardingMUXALUi1),
        .mainALUOutput(mainALUOutput),
        .zero(zero),
        .ALUControlOutput(ALUControlOutput),
        .regDstOutput(regDstOutput),
        .PIPE_IDEX_OUT_CSignal_EX_RegDst(PIPE_IDEX_OUT_CSignal_EX_RegDst),
        .PIPE_IDEX_OUT_CSignal_EX_ALUSrc(PIPE_IDEX_OUT_CSignal_EX_ALUSrc),
        .PIPE_IDEX_OUT_CSignal_WB_MemtoReg(PIPE_IDEX_OUT_CSignal_WB_MemtoReg),
        .PIPE_IDEX_OUT_CSignal_WB_RegWrite(PIPE_IDEX_OUT_CSignal_WB_RegWrite),
        .PIPE_IDEX_OUT_CSignal_MEM_MRead(PIPE_IDEX_OUT_CSignal_MEM_MRead),
        .PIPE_IDEX_OUT_CSignal_MEM_MWrite(PIPE_IDEX_OUT_CSignal_MEM_MWrite),
        .PIPE_IDEX_OUT_CSignal_MEM_Branch(PIPE_IDEX_OUT_CSignal_MEM_Branch),
        .PIPE_IDEX_OUT_CSignal_EX_ALUOp(PIPE_IDEX_OUT_CSignal_EX_ALUOp),
        .PIPE_EXMEM_OUT_CSignal_WB_MemtoReg(PIPE_EXMEM_OUT_CSignal_WB_MemtoReg),
        .PIPE_EXMEM_OUT_CSignal_WB_RegWrite(PIPE_EXMEM_OUT_CSignal_WB_RegWrite),
        .PIPE_EXMEM_OUT_CSignal_MEM_MRead(PIPE_EXMEM_OUT_CSignal_MEM_MRead),
        .PIPE_EXMEM_OUT_CSignal_MEM_MWrite(PIPE_EXMEM_OUT_CSignal_MEM_MWrite),
        .PIPE_EXMEM_OUT_CSignal_MEM_Branch(PIPE_EXMEM_OUT_CSignal_MEM_Branch),
        .PIPE_EXMEM_OUT_BranchALUOutput(PIPE_EXMEM_OUT_BranchALUOutput),
        .PIPE_EXMEM_OUT_Zero(PIPE_EXMEM_OUT_Zero),
        .PIPE_EXMEM_OUT_MainALUOutput(PIPE_EXMEM_OUT_MainALUOutput),
        .PIPE_EXMEM_OUT_ReadData2(PIPE_EXMEM_OUT_ReadData2),
        .PIPE_EXMEM_OUT_RegDstOutput(PIPE_EXMEM_OUT_RegDstOutput),
        .dataMemoryOutput(dataMemoryOutput),
        .branchGateOutput(branchGateOutput),
        .PIPE_MEMWB_OUT_CSignal_MemtoReg(PIPE_MEMWB_OUT_CSignal_MemtoReg),
        .PIPE_MEMWB_OUT_CSignal_RegWrite(PIPE_MEMWB_OUT_CSignal_RegWrite),
        .PIPE_MEMWB_DataMemoryOutput(PIPE_MEMWB_DataMemoryOutput),
        .PIPE_MEMWB_MainALUOutput(PIPE_MEMWB_MainALUOutput),
        .PIPE_MEMWB_RegDstOutput(PIPE_MEMWB_RegDstOutput),
        .memtoRegOutput(memtoRegOutput),
        .clk(clk)
    );
	initial begin
		$dumpfile("testbench.vcd");
		$dumpvars(0,testBench);
		f = $fopen("output.txt", "w");
		$fclose(f);
	end
	always@(clk) begin
	#50
		f = $fopen("output.txt","a");
		$fwrite(f,"\n******************************************************************\n");
		$fwrite(f,"ForwardingUnit      :        ForwardA      | %b\n", CSignal_ForwardingMUX_ALUi0);
		$fwrite(f,"ForwardingUnit      :        ForwardB      | %b\n\n", CSignal_ForwardingMUX_ALUi1);
		$fwrite(f,"Clock               :          Clock       | %b\n", clk);
		$fwrite(f,"Stage IF:           :           PC         | %b\n", PCOutput);
		$fwrite(f,"Stage IF:           :         PC + 4       | %b\n", ALUPCPlus4Output);
		$fwrite(f,"Stage IF:           :       Instruction    | %b\n\n", instruction);

		$fwrite(f,"Stage ID:           :        PC Write      | %b\n", PCWrite);
		$fwrite(f,"Stage ID:           :       IFID Write     | %b\n", IF_ID_Write);
		$fwrite(f,"Stage ID:           :        hazardMux OP  | %b\n", MUX_ID_EX_Write);
		$fwrite(f,"Stage ID:           :         PC + 4       | %b\n", PIPE_IFID_ALUPCPlus4Output);
		$fwrite(f,"Stage ID:           :       Instruction    | %b\n", PIPE_IFID_Instruction);
		$fwrite(f,"Stage ID:           :       Read Data 1    | %b\n", readData1);
		$fwrite(f,"Stage ID:           :       Read Data 2    | %b\n", readData2);
		$fwrite(f,"Stage ID:           :       Sign Extend    | %b\n", signExtendOutput);
		$fwrite(f,"Stage ID:           :            RS        | %b\n", PIPE_IFID_Instruction[25:21]);
		$fwrite(f,"Stage ID:           :            RT        | %b\n", PIPE_IFID_Instruction[20:16]);
		$fwrite(f,"Stage ID:           :            RD        | %b\n", PIPE_IFID_Instruction[15:11]);
		$fwrite(f,"Stage ID:           :    C_Signal RegDst   | %b\n", CSignal_RegDst);
		$fwrite(f,"Stage ID:           :    C_Signal ALUSrc   | %b\n", CSignal_ALUSrc);
		$fwrite(f,"Stage ID:           :    C_Signal MemtoReg | %b\n", CSignal_MemtoReg);
		$fwrite(f,"Stage ID:           :    C_Signal RegWrite | %b\n", CSignal_RegWrite);
		$fwrite(f,"Stage ID:           :    C_Signal MemRead  | %b\n", CSignal_MemRead);
		$fwrite(f,"Stage ID:           :    C_Signal MemWrite | %b\n", CSignal_MemWrite);
		$fwrite(f,"Stage ID:           :    C_Signal Branch   | %b\n", CSignal_Branch);
		$fwrite(f,"Stage ID:           :    C_Signal ALUOp    | %b\n\n", CSignal_ALUOp);

		$fwrite(f,"Stage EX:           :         PC + 4       | %b\n", PIPE_IDEX_OUT_ALUPCPlus4Output);
		$fwrite(f,"Stage EX:           :       ReadData1      | %b\n", PIPE_IDEX_OUT_ReadData1);
		$fwrite(f,"Stage EX:           :       ReadData2      | %b\n", PIPE_IDEX_OUT_ReadData2);
		$fwrite(f,"Stage EX:           :       SignExtend     | %b\n", PIPE_IDEX_OUT_SignExt);
		$fwrite(f,"Stage EX:           :           RS         | %b\n", PIPE_IDEX_OUT_RS);
		$fwrite(f,"Stage EX:           :           RT         | %b\n", PIPE_IDEX_OUT_RT);
		$fwrite(f,"Stage EX:           :           RD         | %b\n", PIPE_IDEX_OUT_RD);
		$fwrite(f,"Stage EX:           :           SLL        | %b\n", sllOutput);
		$fwrite(f,"Stage EX:           :       Branch ALU     | %b\n", branchALUOutput);
		$fwrite(f,"Stage EX:           :       MUX ALU Src    | %b\n", ALUSrcOutput);
		$fwrite(f,"Stage EX:           :     MUX Forward i0   | %b\n", forwardingMUXALUi0);
		$fwrite(f,"Stage EX:           :     MUX Forward i1   | %b\n", forwardingMUXALUi1);
		$fwrite(f,"Stage EX:           :       Main ALU       | %b\n", mainALUOutput);
		$fwrite(f,"Stage EX:           :          Zero        | %b\n", zero);
		$fwrite(f,"Stage EX:           :  ALU Control Output  | %b\n", ALUControlOutput);
		$fwrite(f,"Stage EX:           :       MUX RegDst     | %b\n", regDstOutput);
		$fwrite(f,"Stage EX:           :   C_Signal RegDst    | %b\n", PIPE_IDEX_OUT_CSignal_EX_RegDst);
		$fwrite(f,"Stage EX:           :   C_Signal ALUSrc    | %b\n", PIPE_IDEX_OUT_CSignal_EX_ALUSrc);
		$fwrite(f,"Stage EX:           :   C_Signal MemtoReg  | %b\n", PIPE_IDEX_OUT_CSignal_WB_MemtoReg);
		$fwrite(f,"Stage EX:           :   C_Signal RegWrite  | %b\n", PIPE_IDEX_OUT_CSignal_WB_RegWrite);
		$fwrite(f,"Stage EX:           :   C_Signal MemRead   | %b\n", PIPE_IDEX_OUT_CSignal_MEM_MRead);
		$fwrite(f,"Stage EX:           :   C_Signal MemWrite  | %b\n", PIPE_IDEX_OUT_CSignal_MEM_MWrite);
		$fwrite(f,"Stage EX:           :   C_Signal Branch    | %b\n", PIPE_IDEX_OUT_CSignal_MEM_Branch);
		$fwrite(f,"Stage EX:           :   C_Signal ALUOp     | %b\n\n", PIPE_IDEX_OUT_CSignal_EX_ALUOp);

		$fwrite(f,"Stage MEM:           :   C_Signal MemtoReg  | %b\n", PIPE_EXMEM_OUT_CSignal_WB_MemtoReg);
		$fwrite(f,"Stage MEM:           :   C_Signal RegWrite  | %b\n", PIPE_EXMEM_OUT_CSignal_WB_RegWrite);
		$fwrite(f,"Stage MEM:           :   C_Signal MemRead   | %b\n", PIPE_EXMEM_OUT_CSignal_MEM_MRead);
		$fwrite(f,"Stage MEM:           :   C_Signal MemWrite  | %b\n", PIPE_EXMEM_OUT_CSignal_MEM_MWrite);
		$fwrite(f,"Stage MEM:           :   C_Signal Branch    | %b\n", PIPE_EXMEM_OUT_CSignal_MEM_Branch);
		$fwrite(f,"Stage MEM:           :   BranchALUOutput    | %b\n", PIPE_EXMEM_OUT_BranchALUOutput);
		$fwrite(f,"Stage MEM:           :   Zero               | %b\n", PIPE_EXMEM_OUT_Zero);
		$fwrite(f,"Stage MEM:           :   MainALUOutput      | %b\n", PIPE_EXMEM_OUT_MainALUOutput);
		$fwrite(f,"Stage MEM:           :   ReadData2          | %b\n", PIPE_EXMEM_OUT_ReadData2);
		$fwrite(f,"Stage MEM:           :   RegDstOutput       | %b\n", PIPE_EXMEM_OUT_RegDstOutput);
		$fwrite(f,"Stage MEM:           :   Data Memory Output | %b\n", dataMemoryOutput);
		$fwrite(f,"Stage MEM:           :   Branch Gate Output | %b\n\n", branchGateOutput);

		$fwrite(f,"Stage WB:           :   C_Signal MemtoReg   | %b\n", PIPE_MEMWB_OUT_CSignal_MemtoReg);
		$fwrite(f,"Stage WB:           :   C_Signal RegWrite   | %b\n", PIPE_MEMWB_OUT_CSignal_RegWrite);
		$fwrite(f,"Stage WB:           :   Data Memory Output  | %b\n", PIPE_MEMWB_DataMemoryOutput);
		$fwrite(f,"Stage WB:           :     Main ALU Result   | %b\n", PIPE_MEMWB_MainALUOutput);
		$fwrite(f,"Stage WB:           :     Reg Dst Output    | %b\n", PIPE_MEMWB_RegDstOutput);
		$fwrite(f,"Stage WB:           :   Mem to Reg Output   | %b\n\n", memtoRegOutput);

		$fclose(f);
	end
	//Aqui inicia-se a alteração dos valores do clock
	initial begin
        //Inicializando PC com 0
        resetManual = 0;
        #100 pcInManual = 0;
        #200 clk = 1;
        //Começando a Endereçar as instruções
        #100 pcInManual = 4;
        #200 clk = 0;
        #100 pcInManual = 8;
        #200 clk = 1;
        #100 pcInManual = 12;
        #200 clk = 0;
        #100 pcInManual = 32'bx;
        #200 clk = 1;
        #100
        #200 clk = 0;
        #100
        #200 clk = 1;
        #100
        #200 clk = 0;
        #100
        #200 clk = 1;
	end
endmodule
