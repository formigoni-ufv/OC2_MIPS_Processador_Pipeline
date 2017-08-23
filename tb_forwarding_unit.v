module tb_forwarding_unit(
	output wire[1:0] ForwadA,
	output wire[1:0] ForwadB
);
        reg[31:0] ID_EX_rs
        reg[31:0] ID_EX_rt;
        reg[31:0] EX_MEM_rd;
        reg[31:0] MEM_WB_rd;
        reg EX_MEM_EscreveReg;
        reg MEM_WB_EscreveReg;
        forwarding_unit fu(
        .ID_EX_rs(ID_EX_rs),
        .ID_EX_rt(ID_EX_rt),
        .EX_MEM_rd(EX_MEM_rd),
        .MEM_WB_rd(MEM_WB_rd),
        .EX_MEM_EscreveReg(EX_MEM_EscreveReg),
        .MEM_WB_EscreveReg(MEM_WB_EscreveReg),
        .ForwadA(ForwadA),
        .ForwadB(ForwadB)
        );
        initial begin
        // Caso Forward A00 B00 por EscreveReg
            #8 ID_EX_rs <= 1
            ID_EX_rt <= 2
            EX_MEM_rd <= 3
            MEM_WB_rd <= 4
            EX_MEM_EscreveReg <= 0
            MEM_WB_EscreveReg <= 0
        // Caso Forward A00 B00 por EX_MEM_rd = 0
            #8 ID_EX_rs <= 1
            ID_EX_rt <= 2
            EX_MEM_rd <= 0
            MEM_WB_rd <= 1
            EX_MEM_EscreveReg <= 1
            MEM_WB_EscreveReg <= 0
        // Caso Forward A00 B00 por MEM_WB_rd = 0
            #8 ID_EX_rs <= 1
            ID_EX_rt <= 2
            EX_MEM_rd <= 1
            MEM_WB_rd <= 0
            EX_MEM_EscreveReg <= 0
            MEM_WB_EscreveReg <= 1
        // Caso Forward A00 B00 por MEM_WB_rd != ID_EX_rs != EX_MEM_rd e MEM_WB_rd != ID_EX_rt != EX_MEM_rd
            #8 ID_EX_rs <=1
            ID_EX_rt <= 2
            EX_MEM_rd <= 3
            MEM_WB_rd <= 4
            EX_MEM_EscreveReg <= 1
            MEM_WB_EscreveReg <= 1
        // Caso Forward A01 B00
            #8 ID_EX_rs <= 1
            ID_EX_rt <= 2
            EX_MEM_rd <= 3
            MEM_WB_rd <= 1
            EX_MEM_EscreveReg <= 0
            MEM_WB_EscreveReg <= 1
        // Caso Forward A10 B00
            #8 ID_EX_rs <= 1
            ID_EX_rt <= 2
            EX_MEM_rd <= 1
            MEM_WB_rd <= 3
            EX_MEM_EscreveReg <= 1
            MEM_WB_EscreveReg <= 0
        // Caso Forward A00 B01
            #8 ID_EX_rs <= 1
            ID_EX_rt <= 2
            EX_MEM_rd <= 3
            MEM_WB_rd <= 2
            EX_MEM_EscreveReg <= 0
            MEM_WB_EscreveReg <= 1
        // Caso Forward A00 B10
            #8 ID_EX_rs <= 1
            ID_EX_rt <= 2
            EX_MEM_rd <= 2
            MEM_WB_rd <= 3
            EX_MEM_EscreveReg <= 1
            MEM_WB_EscreveReg <= 0
        // Caso Forward MEM + EX
            #8 ID_EX_rs <= 1
            ID_EX_rt <= 2
            EX_MEM_rd <= 1
            MEM_WB_rd <= 2
            EX_MEM_EscreveReg <= 1
            MEM_WB_EscreveReg <= 1
            #20 $finish
        end
endmodule
