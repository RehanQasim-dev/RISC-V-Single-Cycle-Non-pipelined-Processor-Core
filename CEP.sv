module refreshcounter(input clk,rst,output [2:0]counter);
logic [19:0]tim_divider;
always_ff@(posedge clk) begin
		if(rst) tim_divider[19:0]<= #1 20'd0;
		else tim_divider<= #1 tim_divider+1'b1;
	end
	assign counter[2:0]=tim_divider[19:17];
endmodule
module anodecontroller(input logic [2:0]counter,write,output logic [7:0]s);
	always_comb begin
		case({write,counter})
		4'b1000:s=8'b11111110;
		4'b1001:s=8'b11111101;
		4'b1010:s=8'b11111011;
		4'b1011:s=8'b11110111;
		4'b1100:s=8'b11101111;
		4'b1101:s=8'b11011111;
		4'b1110:s=8'b10111111;
		4'b1111:s=8'b01111111;
		default:s=8'b11111111;
	    endcase
	  end
endmodule
module Register(input logic [3:0]d,input logic write,input logic clk,output logic [3:0]q);
	always_ff @(posedge clk)
			if (write) q<= #1 d;
endmodule
module sevenseg(input logic [3:0] data,output logic [6:0] segments);
always_comb begin
case (data)
// abc_defg
4'h0: segments = 7'b1000_000;
4'h1: segments = 7'b1111_001;
4'h2: segments = 7'b0100_100;
4'h3: segments = 7'b0110_000;
4'h4: segments = 7'b0011_001;
4'h5: segments = 7'b0010_010;
4'h6: segments = 7'b0000_010;
4'h7: segments = 7'b1111_000;
4'h8: segments = 7'b0000_000;
4'h9: segments = 7'b0010_000;
4'ha: segments = 7'b0001_000;
4'hb: segments = 7'b0000_011;
4'hc: segments = 7'b1000_110;
4'hd: segments = 7'b0100_001;
4'he: segments = 7'b0000_110;
4'hf: segments = 7'b0001_110;
endcase;
end;
endmodule
module mux8x1(input logic [2:0]selector,input logic [3:0]in0,in1,in2,in3,in4,in5,in6,in7,output logic [3:0]out);
		always_comb
		begin 
            case(selector)
            3'd0:out=in0;
            3'd1:out=in1;
            3'd2:out=in2;
            3'd3:out=in3;
			3'd4:out=in4;
			3'd5:out=in5;
			3'd6:out=in6;
			3'd7:out=in7;
            endcase
	   end
endmodule
module assign7(input logic [31:0]num,input logic write,rst,input logic clk,output logic [6:0]y,output logic [7:0]s_ctrl);
		logic [3:0]o_of_mux;
		logic [3:0]o_ff[7:0];
		logic [2:0] ref_counter;
	Register	R1(num[3:0],write,clk,o_ff[0]);
	Register	R2(num[7:4],write,clk,o_ff[1]);
	Register	R3(num[11:8],write,clk,o_ff[2]);
	Register	R4(num[15:12],write,clk,o_ff[3]);
	Register	R5(num[19:16],write,clk,o_ff[4]);
	Register	R6(num[23:20],write,clk,o_ff[5]);
	Register	R7(num[27:24],write,clk,o_ff[6]);
	Register	R8(num[31:28],write,clk,o_ff[7]);
	refreshcounter	counter(clk,rst,ref_counter);
	mux8x1 		mux(ref_counter,o_ff[0],o_ff[1],o_ff[2],o_ff[3],o_ff[4],o_ff[5],
				o_ff[6],o_ff[7],o_of_mux);
	sevenseg	segment(o_of_mux,y);
	anodecontroller	ANODE(ref_counter,write,s_ctrl);				
endmodule
module clk_div(input logic clk,rst,output logic slow_clk);
	logic  [2:0]divider;
	always_ff @(posedge clk) begin
	if(rst) divider<=3'd0;
	else divider<=divider+1;
	end
	assign slow_clk=divider[2];
endmodule
		
module mux2x1(input logic [31:0]x1,x2,input logic s,output logic [31:0]y);
		always_comb begin
			case(s)
			0:y=x1;
			1:y=x2;
			endcase
		end
endmodule

module mux7x1(input logic [31:0]x1,x2,x3,x4,x5,x6,x7,input logic [2:0]s,output logic [31:0]y);
		always_comb begin
			case(s)
			3'b000:y=x1;
			3'b001:y=x2;
			3'b010:y=x3;
			3'b011:y=x4;
			3'b100:y=x5;
			3'b101:y=x6;
			3'b110:y=x7;
			3'b111:y=32'bX;
			endcase
		end
endmodule
module ALU(input logic [31:0]a,b,input logic [2:0] ALUctrl,output logic [31:0]result,output logic zero);
	logic [31:0]aand,oor,xxor,A_S,SLT,SLTU,t;
	logic [31:0]mux1_o;
	logic C_out,N,V,W,C;//Flags
	assign aand=a&b;
	assign oor=a|b;
	assign xxor=a^b;
	assign t=~b;
	mux2x1 mux1(b,t,ALUctrl[0],mux1_o);
	assign {C_out,A_S}=a+mux1_o+ALUctrl[0];
	//taking Nor of output of adder
	assign zero=~|result;
	assign N=A_S[31];
	assign C=~C_out;
	assign SLTU={30'd0,C};
	assign V=~ALUctrl[1]&(A_S[31]^a[31])&(~(a[31]^b[31]^ALUctrl[0]));
	assign W=A_S[31]^V;
	assign SLT={30'd0,W};
	mux7x1 mux2(A_S,A_S,xxor,SLT,oor,SLTU,aand,ALUctrl,result);
	
endmodule


module Register_file(input logic rst,input logic [4:0]rs1,rs2,rd,input logic reg_write,clk,input logic [31:0]wd,output logic [31:0]rd1,rd2,write,gcd);
	logic [31:0] mem[31:0];
	integer i;
	always_comb begin
		rd1=mem[rs1];
		rd2=mem[rs2];
	end
	assign write=mem[6];
	assign gcd=mem[4];
	always_ff @(posedge clk or posedge rst)
		if(rst) begin
		for (i=0;i<=31;i=i+1)
			mem[i]=32'd0;
		end
		else if(reg_write) mem[rd]<= #1 wd;
endmodule

module instruc_mem(input logic [31:0]ad,output logic [31:0]rd);
	logic [31:0] mem[31:0];
	initial begin
	$readmemh("instructions.txt",mem);
	end
	assign rd=mem[ad[6:2]];
endmodule
module data_mem(input logic [31:0]ad,output logic [31:0]rd);
	logic [31:0] mem[31:0];
	initial begin
	$readmemh("data.txt",mem);
	end
	assign rd=mem[ad[4:0]];
endmodule
module ALU_decoder(input logic [2:0]func3,input logic [1:0]ALUop,input logic op5,func7,output logic [2:0]ALUctrl);
	always_comb begin
	casex({ALUop,func3,op5,func7})
	7'b00XXXXX:ALUctrl=3'b000;
	7'b01XXXXX:ALUctrl=3'b001;
	7'b1000000:ALUctrl=3'b000;
	7'b1000001:ALUctrl=3'b000;
	7'b1000010:ALUctrl=3'b000;
	7'b1000011:ALUctrl=3'b001;
	7'b10100XX:ALUctrl=3'b010;
	7'b10010XX:ALUctrl=3'b011;
	7'b10110XX:ALUctrl=3'b100;
	7'b10111XX:ALUctrl=3'b110;
	7'b10011XX:ALUctrl=3'b101;
	default:  ALUctrl=3'bXXX;
	endcase
	end
endmodule
module MAIN_decoder(input logic [6:0]opcode,input logic func3,zero,output logic RegWrite,PCsrc,ALUSrc,ImmSrc,ResultSrc,output logic [1:0]ALUOp);
	logic Branch;
	always_comb begin
	case(opcode)
	7'b0000011:{RegWrite,ImmSrc,ALUSrc,ResultSrc,Branch,ALUOp}=7'b1011000;
	7'b0110011:{RegWrite,ImmSrc,ALUSrc,ResultSrc,Branch,ALUOp}=7'b1X00010;
	7'b1100011:{RegWrite,ImmSrc,ALUSrc,ResultSrc,Branch,ALUOp}=7'b010X101;
	7'b0010011:{RegWrite,ImmSrc,ALUSrc,ResultSrc,Branch,ALUOp}=7'b1010010;
	endcase
	end
	always_comb begin
	casex({zero,Branch,func3})
	3'bX0X:PCsrc=0;
	3'b010:PCsrc=0;
	3'b011:PCsrc=1;
	3'b110:PCsrc=1;
	3'b111:PCsrc=0;
	endcase
	end
endmodule
module Extend(input logic [31:7]instr,input logic ImmSrc,output logic [31:0]ImmExt);
	always_comb begin 
	case(ImmSrc)
	0:ImmExt={{20{instr[31]}},instr[31:20]};	//I type and load
	1:ImmExt={{20{instr[31]}},instr[7],instr[30:25],instr[11:8],1'b0};//branch
	endcase
	end
endmodule 
module CEP(input logic clk,	rst,output logic [6:0]y,output logic [7:0]s_ctrl );
	logic [31:0]SrcA,SrcB,WD,ALUresult,ReadData;
	logic [31:0]PC;
	logic [31:0]instr,PCNext,PCPlus4;
	logic [31:0]rd2;logic RegWrite;
	logic [31:0]ImmExt,PCTarget,write,gcd;
	logic PCSrc,ResultSrc,Zero,ALUSrc,ImmSrc;
	logic [1:0]ALUOp;logic [2:0]ALUctrl;
	logic slow_clk;
	clk_div clk_div(clk,rst,slow_clk);
	//PC counter
	always_ff @(posedge slow_clk or posedge rst) begin
	   if(rst) PC<= #1 32'd0;
	   else PC<= #1 PCNext;
	end
	mux2x1 mux1(PCPlus4,PCTarget,PCSrc,PCNext);
	
	assign PCPlus4=PC+4;
	
	instruc_mem memory1(PC,instr);
	
	Register_file file(rst,instr[19:15],instr[24:20],instr[11:7],RegWrite,slow_clk,WD,SrcA,rd2,write,gcd);
	
	mux2x1 mux2(rd2,ImmExt,ALUSrc,SrcB);
	
	Extend E(instr[31:7],ImmSrc,ImmExt);
	
	ALU ALU(SrcA,SrcB,ALUctrl,ALUresult,Zero);
	
	data_mem memory2(ALUresult,ReadData);
	
	mux2x1 mux3(ALUresult,ReadData,ResultSrc,WD);
	
	assign PCTarget=ImmExt+PC;
	
	MAIN_decoder decoder1(instr[6:0],instr[12],Zero,RegWrite,PCSrc,ALUSrc,ImmSrc,ResultSrc,ALUOp);
	
	ALU_decoder  decoder2(instr[14:12],ALUOp,instr[5],instr[30],ALUctrl);
	
	assign7 assignn(gcd,write[0],rst,clk,y,s_ctrl);
endmodule
	