 
module tb_hazard_detection_unit(
    output wire PCWrite,
    output wire IF_ID_Write,
    output wire Mux_ID_EX_Write
);
    reg ID_EX_LeMem;
    reg[31:0] ID_EX_registradorRt;
    reg[31:0] IF_ID_registradorRs;
    reg[31:0] IF_ID_registradorRt;
    hazard_detection_unit hdu(
    .ID_EX_LeMem(ID_EX_LeMem),
    .ID_EX_registradorRt(ID_EX_registradorRt),
    .IF_ID_registradorRs(IF_ID_registradorRs),
    .IF_ID_registradorRt(IF_ID_registradorRt),
    .PCWrite(PCWrite),
    .IF_ID_Write(IF_ID_Write),
    .Mux_ID_EX_Write(Mux_ID_EX_Write)
 );
    initial begin
    //LeMem inativo -> espera-se que nao dẽ stall
        #8 ID_EX_LeMem <= 0;
        IF_ID_registradorRt <= 4;
        IF_ID_registradorRs <= 5;
        ID_EX_registradorRt <= 5;
    //IF/ID.Rs = ID/EX.Rt -> espera-se que dê stall
        #8 ID_EX_LeMem <= 1;
        IF_ID_registradorRt <= 4;
        IF_ID_registradorRs <= 5;
        ID_EX_registradorRt <= 5;
    //IF/ID.Rs != IF/ID.Rt != ID/EX.Rt   -> espera-se que nao de stall
        #8 ID_EX_LeMem <= 1;
        IF_ID_registradorRt <= 4;
        IF_ID_registradorRs <= 5;
        ID_EX_registradorRt <= 6;
    //IF/ID.Rt = ID/EX.Rt -> espera-se que dê stall
        #8 ID_EX_LeMem <= 1;
        IF_ID_registradorRt <= 5;
        IF_ID_registradorRs <= 4;
        ID_EX_registradorRt <= 5;
    //IF/ID.Rs = IF/ID.Rt = ID/EX.Rt  -> espera-se que dê stall
        #8 ID_EX_LeMem <= 1;
        IF_ID_registradorRt <= 4;
        IF_ID_registradorRs <= 4;
        ID_EX_registradorRt <= 4;
        #8 ID_EX_LeMem <= 0;
    //LeMem inativo -> espera-se que nao dẽ stall
        IF_ID_registradorRt <= 4;
        IF_ID_registradorRs <= 5;
        ID_EX_registradorRt <= 6;
        #20 $finish
    end
endmodule
