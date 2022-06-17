module modulator #(
    parameter TICKS = 100
)(
    input clock,
    output modulator
);

    reg [31:0] counter2 = 0;
    reg modulation = 0;

    always @(posedge clock) begin
        if (counter2 < (TICKS/2)) begin
            counter2 <= counter2 + 1;
        end else begin
            modulation = !modulation;
            counter2 <= 0;
        end
    end

    assign modulator = modulation;

endmodule