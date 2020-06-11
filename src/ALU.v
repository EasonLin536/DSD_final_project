module ALU(ctrl, 
           in_0, 
           in_1,
           result,
           Zero
    );
    input       [3:0] ctrl;
    input      [31:0] in_0, in_1;
    output reg [31:0] result;
    output reg        Zero;
    always@(*) begin
        case(ctrl)
            4'b0000: begin // and
                result = in_0 & in_1;
            end
            4'b0001: begin // or
                result = in_0 | in_1;
            end
            4'b0010: begin // add
                result = in_0 + in_1;
            end
            4'b0011: begin // bne sub
                result = in_0 - in_1;
            end
            4'b0110: begin // sub
                result = in_0 - in_1;
            end
            4'b0111: begin // slt
                result = ($signed(in_0) < $signed(in_1))? 1 : 0;
            end
            4'b1000: begin // nor
                result = ~(in_0 | in_1);
            end
            4'b1001: begin // xor
                result = in_0 ^ in_1;
            end
            4'b1010: begin // sll
                result = in_1 << in_0;
            end
            4'b1011: begin // sra
                result = $signed(in_1) >>> in_0;
            end
            4'b1100: begin // srl
                result = in_1 >> in_0;
            end
            default: begin
                result = 32'b0;
            end
        endcase
        Zero = ((result == 32'b0 && ctrl[2]) || (result != 32'b0 && ~ctrl[2])) ? 1 : 0;
    end
endmodule