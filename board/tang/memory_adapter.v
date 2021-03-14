module rom055(
 input [11:0] address,
 input clock,
 output [15:0] q
 );

tang_rom055 rom(
  .addra(address), 
  .doa(q),
  .clka(clock),
  .rsta(1'b0)
);  
	
endmodule
