module REGISTER(
    clk,
    rst_n,
    RegWrite,
    ReadReg1,
    ReadReg2,
    WriteReg,
    WriteData,
    ReadData1,
    ReadData2
);
//==== io Declaration =========================
    input             RegWrite, clk, rst_n;
    input       [4:0] ReadReg1, ReadReg2, WriteReg;
    input      [31:0] WriteData;
    output reg [31:0] ReadData1, ReadData2;

//==== Reg/Wire Declaration ===================
    reg [31:0] Registers[0:31];

    integer i;
    
    always @(posedge clk) begin
        if (!rst_n) begin
            for(i = 0; i <= 31; i = i + 1) Registers[i] <= 32'd0;
        end
        // else if(RegWrite && WriteReg != 0) begin
        //     Registers[WriteReg] <= WriteData;
        // end
        else if (RegWrite) begin
            case(WriteReg)
                0: Registers[0]  <= 0;
		        1: Registers[1]  <= WriteData;
		        2: Registers[2]  <= WriteData;
		        3: Registers[3]  <= WriteData;
		        4: Registers[4]  <= WriteData;
		        5: Registers[5]  <= WriteData;
		        6: Registers[6]  <= WriteData;
		        7: Registers[7]  <= WriteData;
	            8: Registers[8]  <= WriteData;
		        9: Registers[9]  <= WriteData;
		        10:Registers[10] <= WriteData;
		        11:Registers[11] <= WriteData;
		        12:Registers[12] <= WriteData;
		        13:Registers[13] <= WriteData;
		        14:Registers[14] <= WriteData;
		        15:Registers[15] <= WriteData;
		        16:Registers[16] <= WriteData;
		        17:Registers[17] <= WriteData;
		        18:Registers[18] <= WriteData;
		        19:Registers[19] <= WriteData;
		        20:Registers[20] <= WriteData;
		        21:Registers[21] <= WriteData;
		        22:Registers[22] <= WriteData;
		        23:Registers[23] <= WriteData;
		        24:Registers[24] <= WriteData;
		        25:Registers[25] <= WriteData;
		        26:Registers[26] <= WriteData;
		        27:Registers[27] <= WriteData;
		        28:Registers[28] <= WriteData;
		        29:Registers[29] <= WriteData;
		        30:Registers[30] <= WriteData;
		        31:Registers[31] <= WriteData;
                default: Registers[0] <= 0;
            endcase
        end
    end

    // assign ReadData1 = Registers[ReadReg1];
    // assign ReadData2 = Registers[ReadReg2];
    always @(*) begin
        case(ReadReg1)
            0: ReadData1 = Registers[0];
            1: ReadData1 = Registers[1];
            2: ReadData1 = Registers[2];
            3: ReadData1 = Registers[3];
            4: ReadData1 = Registers[4];
            5: ReadData1 = Registers[5];
            6: ReadData1 = Registers[6];
            7: ReadData1 = Registers[7];
            8: ReadData1 = Registers[8];
            9: ReadData1 = Registers[9];
            10:ReadData1 = Registers[10];
            11:ReadData1 = Registers[11];
            12:ReadData1 = Registers[12];
            13:ReadData1 = Registers[13];
            14:ReadData1 = Registers[14];
            15:ReadData1 = Registers[15];
            16:ReadData1 = Registers[16];
            17:ReadData1 = Registers[17];
            18:ReadData1 = Registers[18];
            19:ReadData1 = Registers[19];
            20:ReadData1 = Registers[20];
            21:ReadData1 = Registers[21];
            22:ReadData1 = Registers[22];
            23:ReadData1 = Registers[23];
            23:ReadData1 = Registers[24];
            25:ReadData1 = Registers[25];
            26:ReadData1 = Registers[26];
            27:ReadData1 = Registers[27];
            28:ReadData1 = Registers[28];
            29:ReadData1 = Registers[29];
            30:ReadData1 = Registers[30];
            31:ReadData1 = Registers[31];
            default: ReadData1 = 0;
        endcase
        case(ReadReg2)
            0: ReadData2 = Registers[0];
            1: ReadData2 = Registers[1];
            2: ReadData2 = Registers[2];
            3: ReadData2 = Registers[3];
            4: ReadData2 = Registers[4];
            5: ReadData2 = Registers[5];
            6: ReadData2 = Registers[6];
            7: ReadData2 = Registers[7];
            8: ReadData2 = Registers[8];
            9: ReadData2 = Registers[9];
            10:ReadData2 = Registers[10];
            11:ReadData2 = Registers[11];
            12:ReadData2 = Registers[12];
            13:ReadData2 = Registers[13];
            14:ReadData2 = Registers[14];
            15:ReadData2 = Registers[15];
            16:ReadData2 = Registers[16];
            17:ReadData2 = Registers[17];
            18:ReadData2 = Registers[18];
            19:ReadData2 = Registers[19];
            20:ReadData2 = Registers[20];
            21:ReadData2 = Registers[21];
            22:ReadData2 = Registers[22];
            23:ReadData2 = Registers[23];
            23:ReadData2 = Registers[24];
            25:ReadData2 = Registers[25];
            26:ReadData2 = Registers[26];
            27:ReadData2 = Registers[27];
            28:ReadData2 = Registers[28];
            29:ReadData2 = Registers[29];
            30:ReadData2 = Registers[30];
            31:ReadData2 = Registers[31];
            default: ReadData2 = 0;
        endcase
        if (WriteReg == ReadReg1 && RegWrite && ReadReg1 != 0) ReadData1 = WriteData;
        if (WriteReg == ReadReg2 && RegWrite && ReadReg2 != 0) ReadData2 = WriteData;
    end
endmodule