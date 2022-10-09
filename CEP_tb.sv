module CEP_tb;
	logic clk,rst;logic [6:0]y;logic [7:0]ctrl;
	CEP    CEP(clk,rst,y,ctrl);
	initial begin
	clk=0;
	rst=1;
	@(posedge clk);
	rst=0;
	#1000;
	$stop;
	end
	always #1 clk=~clk;
endmodule
	
	
	