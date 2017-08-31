//Ruan Evangelista Formigoni - 2661
//Vitor Guerra Veloso - 2658

`include "alu_control.v"
`include "andgate.v"
`include "arithmeticlogicunit.v"
`include "datamemory.v"
`include "decoder.v"
`include "instructionmemory.v"
`include "maincontrolunit.v"
`include "multiplexor.v"
`include "programcounter.v"
`include "registerfile.v"
`include "signext.v"
`include "sll.v"
`include "pipelinestages.v"
`include "forwarding_unit.v"

module processor ();

	reg[31:0] i;

	reg clk;

	//Fios da saída ou entrada de módulos, não pipelines
	//IF STAGE
	wire[31:0] PCSrcInput; //Proximo endereco do PC
	wire[31:0] PCOutput; //Saida do PC
	wire[31:0] ALUPCPlus4Output; //Saida da ALU PC + 4
	wire[31:0] instruction; //Instrucao que sai da Instruction Memory
	//ID STAGE
	wire[31:0] readData1; //Dado lido de RS no register file
	wire[31:0] readData2; //Dado lido de RT no register file
	wire[31:0] signExtendOutput; //Saida do modulo de extensao do sinal
	//EX STAGE
	wire[31:0] sllOutput;
	wire[31:0] branchALUOutput;
	wire[31:0] forwardingMUXALUi0;
	wire[31:0] forwardingMUXALUi1;
	wire[31:0] ALUSrcOutput;
	wire[31:0] mainALUOutput;
	wire zero;
	wire[3:0] ALUControlOutput;
	wire[4:0] regDstOutput;
	//MEM STAGE
	wire branchGateOutput;
	wire[31:0] dataMemoryOutput;
	//WB STAGE
	wire[31:0] memtoRegOutput;

	//Sinais do Controle
	wire CSignal_RegWrite; //
	wire CSignal_RegDst; //Sinal para escolher o registrador destino (RT ou RD)
	wire CSignal_ALUSrc; //Sinal para escolher entre o valor com sinal extendido ou o valor contido em RT
	wire CSignal_MemtoReg; //Escolhe se o dado é escrito da memória ou da ALU
	wire CSignal_MemRead; //Habilita leitura memoria
	wire CSignal_MemWrite; //Habilida escrita na memoria
	wire CSignal_Branch; //Habilita o Branch
	wire[1:0] CSignal_ALUOp; //Escolhe a operação da ALU
	wire[1:0] CSignal_ForwardingMUX_ALUi0;
	wire[1:0] CSignal_ForwardingMUX_ALUi1;

	/*****************************************PIPELINED**********************************/
	reg resetManual; //TODO substituir pelo reset chaveado
	reg[31:0] pcInManual;  //TODO remover pelo PCSrcInput do mux, esta variavel garante dont cares

	//Fios de saida da pipeline IF/ID
	wire[31:0] PIPE_IFID_ALUPCPlus4Output;
	wire[31:0] PIPE_IFID_Instruction;

	//Fios de saida da pipeline ID/EX
	wire[31:0] PIPE_IDEX_OUT_ALUPCPlus4Output;
	wire[31:0] PIPE_IDEX_OUT_ReadData1;
	wire[31:0] PIPE_IDEX_OUT_ReadData2;
	wire[31:0] PIPE_IDEX_OUT_SignExt;
	wire[4:0] PIPE_IDEX_OUT_RS;
	wire[4:0] PIPE_IDEX_OUT_RT;
	wire[4:0] PIPE_IDEX_OUT_RD;
	//Sinais de controle da pipeline ID/EX
	wire PIPE_IDEX_OUT_CSignal_EX_ALUSrc;
	wire[1:0] PIPE_IDEX_OUT_CSignal_EX_ALUOp;
	wire PIPE_IDEX_OUT_CSignal_EX_RegDst;
	wire PIPE_IDEX_OUT_CSignal_MEM_Branch;
	wire PIPE_IDEX_OUT_CSignal_MEM_MRead;
	wire PIPE_IDEX_OUT_CSignal_MEM_MWrite;
	wire PIPE_IDEX_OUT_CSignal_WB_RegWrite;
	wire PIPE_IDEX_OUT_CSignal_WB_MemtoReg;

	//Fios de saida da pipeline EX/MEM
	wire[31:0] PIPE_EXMEM_OUT_BranchALUOutput;
	wire       PIPE_EXMEM_OUT_Zero;
	wire[31:0] PIPE_EXMEM_OUT_MainALUOutput;
	wire[31:0] PIPE_EXMEM_OUT_ReadData2;
	wire[4:0]  PIPE_EXMEM_OUT_RegDstOutput;
	//Sinais de controle da pipeline EX/MEM
	wire PIPE_EXMEM_OUT_CSignal_MEM_Branch;
	wire PIPE_EXMEM_OUT_CSignal_MEM_MRead;
	wire PIPE_EXMEM_OUT_CSignal_MEM_MWrite;
	wire PIPE_EXMEM_OUT_CSignal_WB_RegWrite;
	wire PIPE_EXMEM_OUT_CSignal_WB_MemtoReg;

	//Sinais de controle da pipeline MEM/WB
	wire       PIPE_MEMWB_OUT_CSignal_RegWrite;
	wire       PIPE_MEMWB_OUT_CSignal_MemtoReg;
	//Fios de saida da pipeline MEM/WB
	wire[31:0] PIPE_MEMWB_DataMemoryOutput;
	wire[31:0] PIPE_MEMWB_MainALUOutput;
	wire[4:0]  PIPE_MEMWB_RegDstOutput;

	/******************************Instruction Fetch Stage********************************/
	multiplexorPCSrc PCSrc(.i0(ALUPCPlus4Output), .i1(PIPE_EXMEM_OUT_BranchALUOutput), .control(branchGateOutput), .out(PCSrcInput));

	programcounter PC(.clock(clk), .in(pcInManual), .out(PCOutput), .reset(resetManual) );

	instructionmemory InstructionMemory(.addr(PCOutput), .instruction(instruction) );

	arithmeticlogicunit ALUPCPlus4(.A(PCOutput), .B(32'b100), .OP(4'b10), .OUT(ALUPCPlus4Output) );

	PIPE_IF_ID IF_ID(.clk(clk), .PIPEIN_PCPlus4(ALUPCPlus4Output), .PIPEIN_InsMemory(instruction), .PIPEOUT_PCPlus4(PIPE_IFID_ALUPCPlus4Output), .PIPEOUT_InsMemory(PIPE_IFID_Instruction) );

	/******************************Instruction Decode Stage******************************/
	maincontrolunit ControlUnit(.op(PIPE_IFID_Instruction[31:26]), .regDst(CSignal_RegDst), .ALUSrc(CSignal_ALUSrc), .memtoReg(CSignal_MemtoReg), .regWrite(CSignal_RegWrite), .memRead(CSignal_MemRead), .memWrite(CSignal_MemWrite), .branch(CSignal_Branch), .ALUOp1(CSignal_ALUOp[1]), .ALUOp0(CSignal_ALUOp[0]) );

	registerfile RegisterFile(.clk(clk), .reg1addr(PIPE_IFID_Instruction[25:21]), .reg2addr(PIPE_IFID_Instruction[20:16]), .writeRegister(PIPE_MEMWB_RegDstOutput), .writeData(memtoRegOutput), .regWrite(PIPE_MEMWB_OUT_CSignal_RegWrite), .reg1content(readData1),	.reg2content(readData2) );

	signext SignExt(.i0(PIPE_IFID_Instruction[15:0]), .out(signExtendOutput) );


	PIPE_ID_EX ID_EX(
		.clk(clk),
		//DATA VARS IN
		.PIPEIN_PCPlus4(PIPE_IFID_ALUPCPlus4Output),
		.PIPEIN_ReadData1(readData1),
		.PIPEIN_ReadData2(readData2),
		.PIPEIN_SignExt(signExtendOutput),
		.PIPEIN_RS(PIPE_IFID_Instruction[25:21]),
		.PIPEIN_RT(PIPE_IFID_Instruction[20:16]),
		.PIPEIN_RD(PIPE_IFID_Instruction[15:11]),

		//CONTROL VARS IN
		.PIPEIN_EX_ALUSrc(CSignal_ALUSrc),
		.PIPEIN_EX_ALUOp(CSignal_ALUOp),
		.PIPEIN_EX_RegDst(CSignal_RegDst),
		.PIPEIN_MEM_Branch(CSignal_Branch),
		.PIPEIN_MEM_MRead(CSignal_MemRead),
		.PIPEIN_MEM_MWrite(CSignal_MemWrite),
		.PIPEIN_WB_RegWrite(CSignal_RegWrite),
		.PIPEIN_WB_MemtoReg(CSignal_MemtoReg),

		//DATA VARS OUT
		.PIPEOUT_PCPlus4(PIPE_IDEX_OUT_ALUPCPlus4Output),
		.PIPEOUT_ReadData1(PIPE_IDEX_OUT_ReadData1),
		.PIPEOUT_ReadData2(PIPE_IDEX_OUT_ReadData2),
		.PIPEOUT_SignExt(PIPE_IDEX_OUT_SignExt),
		.PIPEOUT_RS(PIPE_IDEX_OUT_RS),
		.PIPEOUT_RT(PIPE_IDEX_OUT_RT),
		.PIPEOUT_RD(PIPE_IDEX_OUT_RD),

		//CONTROL VARS OUT
		.PIPEOUT_EX_ALUSrc(PIPE_IDEX_OUT_CSignal_EX_ALUSrc),
		.PIPEOUT_EX_ALUOp(PIPE_IDEX_OUT_CSignal_EX_ALUOp),
		.PIPEOUT_EX_RegDst(PIPE_IDEX_OUT_CSignal_EX_RegDst),
		.PIPEOUT_MEM_Branch(PIPE_IDEX_OUT_CSignal_MEM_Branch),
		.PIPEOUT_MEM_MRead(PIPE_IDEX_OUT_CSignal_MEM_MRead),
		.PIPEOUT_MEM_MWrite(PIPE_IDEX_OUT_CSignal_MEM_MWrite),
		.PIPEOUT_WB_RegWrite(PIPE_IDEX_OUT_CSignal_WB_RegWrite),
		.PIPEOUT_WB_MemtoReg(PIPE_IDEX_OUT_CSignal_WB_MemtoReg)

	);

	/******************************Instruction Execution Stage******************************/
	shiftlogicalleft SLL(.i0(PIPE_IDEX_OUT_SignExt), .out(sllOutput));

	arithmeticlogicunit branchALU(.A(PIPE_IDEX_OUT_ALUPCPlus4Output), .B(sllOutput), .OP(4'b10), .OUT(branchALUOutput) );

	mainALUForwardingMUX ALUi0(.i0(PIPE_IDEX_OUT_ReadData1), .i1(memtoRegOutput), .i2(PIPE_EXMEM_OUT_MainALUOutput), .control(CSignal_ForwardingMUX_ALUi0), .out(forwardingMUXALUi0) );

	mainALUForwardingMUX ALUi1(.i0(PIPE_IDEX_OUT_ReadData2), .i1(memtoRegOutput), .i2(PIPE_EXMEM_OUT_MainALUOutput), .control(CSignal_ForwardingMUX_ALUi1), .out(forwardingMUXALUi1) );

	multiplexorALUSrc ALUSrcMux(.i0(forwardingMUXALUi1), .i1(PIPE_IDEX_OUT_SignExt), .control(PIPE_IDEX_OUT_CSignal_EX_ALUSrc), .out(ALUSrcOutput) );

	arithmeticlogicunit mainALU(.A(forwardingMUXALUi0), .B(ALUSrcOutput), .OP(ALUControlOutput), .OUT(mainALUOutput), .zero(zero) );

	alu_control ALUControl(.ALUOp(PIPE_IDEX_OUT_CSignal_EX_ALUOp), .funcCode(PIPE_IDEX_OUT_SignExt[5:0]), .aluCtrlOut(ALUControlOutput) );

	multiplexorRegDst regDstMUX(.i0(PIPE_IDEX_OUT_RT), .i1(PIPE_IDEX_OUT_RD), .control(PIPE_IDEX_OUT_CSignal_EX_RegDst), .out(regDstOutput) );

	PIPE_EX_MEM EX_MEM(
		.clk(clk),

		//Control Vars IN
		.PIPEIN_MEM_Branch(PIPE_IDEX_OUT_CSignal_MEM_Branch),
		.PIPEIN_MEM_MRead(PIPE_IDEX_OUT_CSignal_MEM_MRead),
		.PIPEIN_MEM_MWrite(PIPE_IDEX_OUT_CSignal_MEM_MWrite),
		.PIPEIN_WB_RegWrite(PIPE_IDEX_OUT_CSignal_WB_RegWrite),
		.PIPEIN_WB_MemtoReg(PIPE_IDEX_OUT_CSignal_WB_MemtoReg),
		//Data Vars IN
		.PIPEIN_BranchALUOutput(branchALUOutput),
		.PIPEIN_Zero(zero),
		.PIPEIN_ALUResult(mainALUOutput),
		.PIPEIN_ReadData2(PIPE_IDEX_OUT_ReadData2),
		.PIPEIN_RegDstOutput(regDstOutput),

		//Control Vars OUT
		.PIPEOUT_MEM_Branch(PIPE_EXMEM_OUT_CSignal_MEM_Branch),
		.PIPEOUT_MEM_MRead(PIPE_EXMEM_OUT_CSignal_MEM_MRead),
		.PIPEOUT_MEM_MWrite(PIPE_EXMEM_OUT_CSignal_MEM_MWrite),
		.PIPEOUT_WB_RegWrite(PIPE_EXMEM_OUT_CSignal_WB_RegWrite),
		.PIPEOUT_WB_MemtoReg(PIPE_EXMEM_OUT_CSignal_WB_MemtoReg),
		//Data Vars OUT
		.PIPEOUT_BranchALUOutput(PIPE_EXMEM_OUT_BranchALUOutput),
		.PIPEOUT_Zero(PIPE_EXMEM_OUT_Zero),
		.PIPEOUT_ALUResult(PIPE_EXMEM_OUT_MainALUOutput),
		.PIPEOUT_ReadData2(PIPE_EXMEM_OUT_ReadData2),
		.PIPEOUT_RegDstOutput(PIPE_EXMEM_OUT_RegDstOutput)
	);

	forwarding_unit ForwardingUnit(
		.clk(clk),
		.EX_MEM_RegWrite(PIPE_EXMEM_OUT_CSignal_WB_RegWrite),
		.MEM_WB_RegWrite(PIPE_MEMWB_OUT_CSignal_RegWrite),
		.ID_EX_Rs(PIPE_IDEX_OUT_RS),
		.ID_EX_Rt(PIPE_IDEX_OUT_RT),
		.EX_MEM_Rd(PIPE_EXMEM_OUT_RegDstOutput),
		.MEM_WB_Rd(PIPE_MEMWB_RegDstOutput),
		.ForwardA(CSignal_ForwardingMUX_ALUi0),
		.ForwardB(CSignal_ForwardingMUX_ALUi1)
	);

	/******************************Memory Stage******************************/
	andgate gate(.i0(PIPE_EXMEM_OUT_CSignal_MEM_Branch), .i1(PIPE_EXMEM_OUT_Zero), .out(branchGateOutput) );

	datamemory datamem(.clk(clk), .addr(PIPE_EXMEM_OUT_MainALUOutput), .writeData(PIPE_EXMEM_OUT_ReadData2), .memRead(PIPE_EXMEM_OUT_CSignal_MEM_MRead), .memWrite(PIPE_EXMEM_OUT_CSignal_MEM_MWrite), .readData(dataMemoryOutput) );

	PIPE_MEM_WB MEM_WB(
		.clk(clk),

		//Control Vars IN
		.PIPEIN_WB_RegWrite(PIPE_EXMEM_OUT_CSignal_WB_RegWrite),
		.PIPEIN_WB_MemtoReg(PIPE_EXMEM_OUT_CSignal_WB_MemtoReg),
		//Data Vars IN
		.PIPEIN_DataMemoryOutput(dataMemoryOutput),
		.PIPEIN_MainALUOutput(PIPE_EXMEM_OUT_MainALUOutput),
		.PIPEIN_RegDstOutput(PIPE_EXMEM_OUT_RegDstOutput),

		//Control Vars OUT
		.PIPEOUT_WB_RegWrite(PIPE_MEMWB_OUT_CSignal_RegWrite),
		.PIPEOUT_WB_MemtoReg(PIPE_MEMWB_OUT_CSignal_MemtoReg),
		//Data Vars OUT
		.PIPEOUT_DataMemoryOutput(PIPE_MEMWB_DataMemoryOutput),
		.PIPEOUT_MainALUOutput(PIPE_MEMWB_MainALUOutput),
		.PIPEOUT_RegDstOutput(PIPE_MEMWB_RegDstOutput)
	);

	/******************************Write Back Stage******************************/
	multiplexorMemtoReg MemtoReg(.i0(PIPE_MEMWB_MainALUOutput), .i1(PIPE_MEMWB_DataMemoryOutput), .control(PIPE_MEMWB_OUT_CSignal_MemtoReg), .out(memtoRegOutput) );

	/******************************************************************************************************************************************************************************************************/


	//Testing
	//Abrindo arquivo de saida
	integer f;

	initial begin
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

		$fwrite(f,"Stage ID: IF|ID OUT :         PC + 4       | %b\n", PIPE_IFID_ALUPCPlus4Output);
		$fwrite(f,"Stage ID: IF|ID OUT :       Instruction    | %b\n", PIPE_IFID_Instruction);
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

	initial begin
	//Inicializando PC com 0
	#200 resetManual = 0;
	#200 pcInManual = 0;
	#200 clk = 1;
	//Começando a Endereçar as instruções
	#200 pcInManual = 4;
	#200 clk = 0;
	#200 pcInManual = 8;
	#200 clk = 1;
	#200 pcInManual = 32'bx;
	#200 clk = 0;
	#200 clk = 1;
	#200 clk = 0;
	#200 pcInManual = 12;
	#200 clk = 1;
	#200 pcInManual = 32'bx;
	#200 clk = 0;
	#200 clk = 1;
	#200 clk = 0;
	#200 clk = 1;
	#200 pcInManual = 16;
	#200 clk = 0;
	#200 pcInManual = 32'bx;
	#200 clk = 1;
	#200 clk = 0;
	#200 clk = 1;
	#200 clk = 0;
	end
endmodule







/******************************************************************************************Single Cycle**************************************************************************************************************************/
//Modules Instantiation

// andgate gate(.i0(branch), .i1(zero), .out(andGateOut) );
//
// datamemory datamem(.clk(clk), .addr(aluMainOut), .writeData(reg2content), .memRead(memRead), .memWrite(memWrite), .readData(dataMemOut) );
//
// multiplexorMemtoReg dataMemMux(.i0(aluMainOut), .i1(dataMemOut), .control(MemtoReg), .out(memtoRegOut) );
/****************************************************************************************************************************************************************************************************************/

// $fwrite(f, "\n******************************************************************\n");
// $fwrite(f, "Clock               :          Clock       | %b\n", clk);
// $fwrite(f, "Stage IF:           :           PC         | %b\n", PCOutput);
// $fwrite(f, "Stage IF:           :         PC + 4       | %b\n", ALUPCPlus4Output);
// $fwrite(f, "Stage IF:           :       Instruction    | %b\n\n", instruction);
//
// $display("Stage ID: IF|ID OUT :         PC + 4       | %b", PIPE_IFID_ALUPCPlus4Output);
// $display("Stage ID: IF|ID OUT :       Instruction    | %b", PIPE_IFID_Instruction);
// $display("Stage ID:           :       Read Data 1    | %b", readData1);
// $display("Stage ID:           :       Read Data 2    | %b", readData2);
// $display("Stage ID:           :       Sign Extend    | %b", signExtendOutput);
// $display("Stage ID:           :            RT        | %b", PIPE_IFID_Instruction[20:16]);
// $display("Stage ID:           :            RD        | %b", PIPE_IFID_Instruction[15:11]);
// $display("Stage ID:           :    C_Signal RegDst   | %b", CSignal_RegDst);
// $display("Stage ID:           :    C_Signal ALUSrc   | %b", CSignal_ALUSrc);
// $display("Stage ID:           :    C_Signal MemtoReg | %b", CSignal_MemtoReg);
// $display("Stage ID:           :    C_Signal RegWrite | %b", CSignal_RegWrite);
// $display("Stage ID:           :    C_Signal MemRead  | %b", CSignal_MemRead);
// $display("Stage ID:           :    C_Signal MemWrite | %b", CSignal_MemWrite);
// $display("Stage ID:           :    C_Signal Branch   | %b", CSignal_Branch);
// $display("Stage ID:           :    C_Signal ALUOp    | %b\n", CSignal_ALUOp);
//
// $display("Stage EX:           :         PC + 4       | %b", PIPE_IDEX_OUT_ALUPCPlus4Output);
// $display("Stage EX:           :       ReadData1      | %b", PIPE_IDEX_OUT_ReadData1);
// $display("Stage EX:           :       ReadData2      | %b", PIPE_IDEX_OUT_ReadData2);
// $display("Stage EX:           :       SignExtend     | %b", PIPE_IDEX_OUT_SignExt);
// $display("Stage EX:           :           RT         | %b", PIPE_IDEX_OUT_RT);
// $display("Stage EX:           :           RD         | %b", PIPE_IDEX_OUT_RD);
// $display("Stage EX:           :           SLL        | %b", sllOutput);
// $display("Stage EX:           :       Branch ALU     | %b", branchALUOutput);
// $display("Stage EX:           :       MUX ALU Src    | %b", ALUSrcOutput);
// $display("Stage EX:           :       Main ALU       | %b", mainALUOutput);
// $display("Stage EX:           :          Zero        | %b", zero);
// $display("Stage EX:           :  ALU Control Output  | %b", ALUControlOutput);
// $display("Stage EX:           :       MUX RegDst     | %b", regDstOutput);
// $display("Stage EX:           :   C_Signal RegDst    | %b", PIPE_IDEX_OUT_CSignal_EX_RegDst);
// $display("Stage EX:           :   C_Signal ALUSrc    | %b", PIPE_IDEX_OUT_CSignal_EX_ALUSrc);
// $display("Stage EX:           :   C_Signal MemtoReg  | %b", PIPE_IDEX_OUT_CSignal_WB_MemtoReg);
// $display("Stage EX:           :   C_Signal RegWrite  | %b", PIPE_IDEX_OUT_CSignal_WB_RegWrite);
// $display("Stage EX:           :   C_Signal MemRead   | %b", PIPE_IDEX_OUT_CSignal_MEM_MRead);
// $display("Stage EX:           :   C_Signal MemWrite  | %b", PIPE_IDEX_OUT_CSignal_MEM_MWrite);
// $display("Stage EX:           :   C_Signal Branch    | %b", PIPE_IDEX_OUT_CSignal_MEM_Branch);
// $display("Stage EX:           :   C_Signal ALUOp     | %b\n", PIPE_IDEX_OUT_CSignal_EX_ALUOp);
//
// $display("Stage MEM:           :   C_Signal MemtoReg  | %b", PIPE_EXMEM_OUT_CSignal_WB_MemtoReg);
// $display("Stage MEM:           :   C_Signal RegWrite  | %b", PIPE_EXMEM_OUT_CSignal_WB_RegWrite);
// $display("Stage MEM:           :   C_Signal MemRead   | %b", PIPE_EXMEM_OUT_CSignal_MEM_MRead);
// $display("Stage MEM:           :   C_Signal MemWrite  | %b", PIPE_EXMEM_OUT_CSignal_MEM_MWrite);
// $display("Stage MEM:           :   C_Signal Branch    | %b", PIPE_EXMEM_OUT_CSignal_MEM_Branch);
// $display("Stage MEM:           :   BranchALUOutput    | %b", PIPE_EXMEM_OUT_BranchALUOutput);
// $display("Stage MEM:           :   Zero               | %b", PIPE_EXMEM_OUT_Zero);
// $display("Stage MEM:           :   MainALUOutput      | %b", PIPE_EXMEM_OUT_MainALUOutput);
// $display("Stage MEM:           :   ReadData2          | %b", PIPE_EXMEM_OUT_ReadData2);
// $display("Stage MEM:           :   RegDstOutput       | %b", PIPE_EXMEM_OUT_RegDstOutput);
// $display("Stage MEM:           :   Data Memory Output | %b", dataMemoryOutput);
// $display("Stage MEM:           :   Branch Gate Output | %b\n", branchGateOutput);
//
// $display("Stage WB:           :   C_Signal MemtoReg   | %b", PIPE_MEMWB_OUT_CSignal_MemtoReg);
// $display("Stage WB:           :   C_Signal RegWrite   | %b", PIPE_MEMWB_OUT_CSignal_RegWrite);
// $display("Stage WB:           :   Data Memory Output  | %b", PIPE_MEMWB_DataMemoryOutput);
// $display("Stage WB:           :     Main ALU Result   | %b", PIPE_MEMWB_MainALUOutput);
// $display("Stage WB:           :     Reg Dst Output    | %b", PIPE_MEMWB_RegDstOutput);
// $display("Stage WB:           :   Mem to Reg Output   | %b", memtoRegOutput);
