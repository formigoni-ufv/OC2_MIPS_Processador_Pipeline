 module hazard_detection_unit
	(
	input wire ID_EX_LeMem,
    input wire[31:0] ID_EX_registradorRt,
    input wire[31:0] IF_ID_registradorRs,
    input wire[31:0] IF_ID_registradorRt,
    output reg PCWrite,
    output reg IF_ID_Write,
    output reg Mux_ID_EX_Write
	);
	
	always@(*) begin
        if(ID_EX_LeMem &
            (IF_ID_registradorRt == ID_EX_registradorRt  |
            IF_ID_registradorRs == ID_EX_registradorRt)
            ) begin
                PCWrite, IF_ID_Write, Mux_ID_EX_Write = 1;
        end
        else PCWrite, IF_ID_Write, Mux_ID_EX_Write = 0;
	end
endmodule
