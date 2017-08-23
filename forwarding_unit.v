 module forwarding_unit
	(
	input wire[31:0] ID_EX_rs,
	input wire[31:0] ID_EX_rt,
	input wire[31:0] EX_MEM_rd,
	input wire[31:0] MEM_WB_rd,
	input wire EX_MEM_EscreveReg,
	input wire MEM_WB_EscreveReg,
	output reg[1:0] ForwadA,
	output reg[1:0] ForwadB
	);
	reg ForwardMEM;
	initial begin
        ForwardMEM = 0;
	end
	always@(*) begin
        if(EX_MEM_EscreveReg) begin
            if(EX_MEM_rd) begin
                if(EX_MEM_rd == ID_EX_rs) begin
                    ForwadA = 2'b10;
                    ForwardMEM = 1;
                end
                else if(EX_MEM_rd == ID_EX_rt) begin
                    ForwadB = 2'b10;
                    ForwardMEM = 1;
                end
            end
        end
        //Resetar quando trocar ciclo, evitando conflito com o ciclo de clock anterior
        ForwadA = 2'b00;
        ForwadB = 2'b00;
        if(ForwardMEM) ForwardMEM = 0;
        else begin
            if(MEM_WB_EscreveReg) begin
                if(MEM_WB_rd) begin
                    if(MEM_WB_rd == ID_EX_rs) ForwadA = 2'b01;
                    if(MEM_WB_rd == ID_EX_rt) ForwadB = 2'b01;
                end
            end
        end
	end
endmodule
