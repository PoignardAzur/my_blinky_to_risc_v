`include "clockworks.v"

module SOC (
    input  CLK,        // system clock 
    input  RESET,      // reset button
    output [4:0] LEDS, // system LEDs
    input  RXD,        // UART receive
    output TXD         // UART transmit
);
   wire clk;    // internal clock
   wire resetn; // internal reset signal, goes low on reset

   // Clock gearbox (to let you see what happens)
   // and reset circuitry (to workaround an
   // initialization problem with Ice40)
   Clockworks #(
     .SLOW(21) // Divide clock frequency by 2^21
   )CW(
     .CLK(CLK),
     .RESET(RESET),
     .clk(clk),
     .resetn(resetn)
   );

   // A blinker that counts on 5 bits, wired to the 5 LEDs
   reg [3:0] bits = 1;
   always @(posedge clk) begin
      if (!resetn) begin
        bits <= 0;
      end else begin
        bits <= bits << 1 | bits[3];
      end
   end

   assign LEDS = { 1'b1, bits };
   assign TXD  = 1'b0; // not used for now   
endmodule