//Rittvik Saran 2017A8PS0251P
//Arpit Kumar 2018A7PS0272P
//Pranamya Jain 2018A8PS0769P

module control_unit (PCWrite,PCWriteCond1,PCWriteCond2,ImRead,IorD,MemRead,MemWrite,IRWrite,MemtoReg,PCSource,ALUOp_MC,ALUSrcB,ALUSrcA,RegWrite,RegDst,clk,opcode,funct,PC);
input [3:0] opcode,funct;
input [15:0] PC;
input clk;
output reg PCWrite,PCWriteCond1,PCWriteCond2,IorD,MemRead,MemWrite,IRWrite,MemtoReg,RegWrite,RegDst,ImRead;
output reg [1:0] PCSource;
output reg [2:0] ALUOp_MC;
output reg [2:0] ALUSrcB;
output reg [1:0] ALUSrcA;
localparam Idle=5'd0 ,S0=5'd1,S1=5'd2,S2=5'd3,S3=5'd4,S4=5'd5,S5=5'd6,S6=5'd7,S7=5'd8,S8=5'd9,S9=5'd10,S10=5'd11,S11=5'd12,S12=5'd13,S13=5'd14,S14=5'd15,S15=5'd16,S16=5'd17,S17=5'd18,S18=5'd19,S19=5'd20,S20=5'd21,S21=5'd22,S22=5'd23,S23=5'd24;
reg [4:0] present_state=0,next_state;

//sequential block for updating the state
always@(posedge clk)
present_state <= next_state;

//combinational block for calculation of output
always@(*)
begin
//by default making output to be zero initially
ImRead=1'b0;
PCWrite=1'b0;
PCWriteCond1=1'b0;
PCWriteCond2=1'b0;
IorD=1'b0;
MemRead=1'b0;
MemWrite=1'b0;
IRWrite=1'b0;
MemtoReg=1'b0;
PCSource=2'b0;
ALUOp_MC=3'b0;
ALUSrcB=3'b0;
ALUSrcA=2'b0;
RegWrite=1'b0;
RegDst=1'b0;
	case(present_state)
	S0 : begin
		 PCWrite=1'b1; IRWrite=1'b1; PCSource=2'b01; ImRead=1'b1;
		 end
	S1 : begin
		 ALUSrcA=2'b00; ALUSrcB=3'b101;
		 end
	S2 : begin
		 ALUSrcA=2'b01; ALUSrcB=3'b001; ALUOp_MC=3'b000;
		 $display("EX stage: RD = RS1 + RS2");
		 end
	S3 : begin
		 ALUSrcA=2'b10; ALUSrcB=3'b010; ALUOp_MC=3'b000;
		 $display("EX stage: RD = RD + Sign-Extended-Immediate Data");
		 end
	S4 : begin
		 ALUSrcA=2'b10; ALUSrcB=3'b011; ALUOp_MC=3'b000;
		 $display("EX stage: RD = RD + Immediate Data (upper byte filled by zeros)");
		 end
	S5 : begin
		  ALUSrcA=2'b01; ALUSrcB=3'b001; ALUOp_MC=3'b001;
			$display("EX stage: RD = RS1 - RS2");
		 end
	S6 : begin
		 ALUSrcA=2'b10; ALUSrcB=3'b010; ALUOp_MC=3'b001;
		 $display("EX stage: RD = RD - Sign-Extended-Immediate Data");
		 end
	S7 : begin
		 ALUSrcA=2'b10; ALUSrcB=3'b011; ALUOp_MC=3'b001;
		 $display("EX stage: RD = RD - Immediate Data (upper byte filled by zeros)");

		 end
	S8 : begin
		 ALUSrcA=2'b10; ALUSrcB=3'b100; ALUOp_MC=3'b100;
		 $display("EX stage: RD = shift logical left (fill zeros) RD by the immediate number specified");

		 end
	S9 : begin
		  ALUSrcA=2'b10; ALUSrcB=3'b100; ALUOp_MC=3'b100;
			$display("EX stage: RD = shift logical right (fill zeros) RD by the immediate number specified");

		 end
	S10 : begin
		  ALUSrcA=2'b10; ALUSrcB=3'b100; ALUOp_MC=3'b100;
			$display("EX stage: RD = shift right (copy MSB) RD by the immediate number specified");

		 end
	S11 : begin
		 ALUSrcA=2'b01; ALUSrcB=3'b001; ALUOp_MC=3'b010;
		 $display("EX stage: RD = RS1 nand RS2");

		 end
	S12 : begin
		 ALUSrcA=2'b01; ALUSrcB=3'b001; ALUOp_MC=3'b011;
		 $display("EX stage: RD = RS1 or RS2");

		 end
	S13 : begin
		  ALUSrcA=2'b10; ALUSrcB=3'b010; ALUOp_MC=3'b010;
			$display("EX stage: RD = RD nand (sign extended immediate data)");

		 end
	S14 : begin
		  ALUSrcA=2'b10; ALUSrcB=3'b010; ALUOp_MC=3'b011;
			$display("EX stage: RD = RD or (sign extended immediate data)");

		 end
	S15 : begin
		  RegWrite=1'b1;
			$display("MEM stage: ALUout is written to IR[11:8]");
		 end
	S16 : begin
		  ALUSrcA=2'b01; ALUSrcB=3'b001; ALUOp_MC=3'b001; PCSource=2'b10; PCWriteCond1=1'b1;
			$display("EX stage: branch to the address in register RT when RA and RB are equal");
		 end
	S17 : begin
		  ALUSrcA=2'b01; ALUSrcB=3'b001; ALUOp_MC=3'b001; PCSource=2'b10; PCWriteCond2=1'b1;
			$display("EX stage: branch to the address in register RT when RA and RB are unequal");

		 end
	S18 : begin
		  PCSource=2'b00; PCWrite=1'b1;
			$display("EX stage: Jump to PC + sign extended immediate data");

		 end
	S19 : begin
		  ALUSrcA=2'b11; ALUSrcB=3'b110; ALUOp_MC=3'b000;
			$display("EX stage: ALUOut = Reg['10'+IR[9:8]]+sign extended left shift IR[7:0]");
		 end
	S20 : begin
		  IorD=1'b1; MemRead=1'b1; IRWrite=1'b0;
			$display("MEM stage: MDR is wriiten with mem[ALUOut]");
		 end
	S21 : begin
		  MemtoReg=1'b1; RegDst=1'b1; RegWrite=1'b1;
		 end
	S22 : begin
		  ALUSrcA=2'b11; ALUSrcB=3'b110; ALUOp_MC=3'b000;
			$display("EX stage: ALUOut = Reg['10'+IR[9:8]]+sign extended left shift IR[7:0]");
		 end
	S23 : begin
		  IorD=1'b1; MemWrite=1'b1;
			$display("MEM stage: mem[ALUOut] is written with Reg['11'+IR[11:10]]");
		 end
	default : begin
		PCWrite=1'b0;
		PCWriteCond1=1'b0;
		PCWriteCond2=1'b0;
		IorD=1'b0;
		MemRead=1'b0;
		MemWrite=1'b0;
		IRWrite=1'b0;
		MemtoReg=1'b0;
		PCSource=2'b0;
		ALUOp_MC=3'b0;
		ALUSrcB=3'b0;
		ALUSrcA=2'b0;
		RegWrite=1'b0;
		RegDst=1'b0;
		end
	endcase
	/*always@(PCWrite, IRWrite) begin
	if(PCWrite==1'b1)
	$display("new instruction, IF stage=%d", PC);
	if(IRWrite==1'b1)
	$display("ID stage. Instruction decoded with - %b", opcode);
	end*/
end

//combinational block for calculation of next state
always@(*)
begin
next_state = present_state;
	case(present_state)
	Idle:next_state = S0;
	S0: next_state = S1;
	S1: begin
		if(opcode==4'b1000)
		next_state = S2;
		if(opcode==4'b1001)
		next_state = S3;
		if(opcode==4'b1010)
		next_state = S4;
		if(opcode==4'b1100)
		next_state = S5;
		if(opcode==4'b1101)
		next_state = S6;
		if(opcode==4'b1110)
		next_state = S7;
		if(opcode==4'b0000 & funct==4'b0001)
		next_state = S8;
		if(opcode==4'b0000 & funct==4'b0010)
		next_state = S9;
		if(opcode==4'b0000 & funct==4'b0011)
		next_state = S10;
		if(opcode==4'b1011)
		next_state = S11;
		if(opcode==4'b1111)
		next_state = S12;
		if(opcode==4'b0111)
		next_state = S13;
		if(opcode==4'b0110)
		next_state = S14;
		if(opcode==4'b0100)
		next_state = S16;
		if(opcode==4'b0101)
		next_state = S17;
		if(opcode==4'b0011)
		next_state = S18;
		if(opcode==4'b0001)
		next_state = S19;
		if(opcode==4'b0010)
		next_state = S22;
		end
	S2: next_state = S15;
	S3: next_state = S15;
	S4: next_state = S15;
	S5: next_state = S15;
	S6: next_state = S15;
	S7: next_state = S15;
	S8: next_state = S15;
	S9: next_state = S15;
	S10: next_state = S15;
	S11: next_state = S15;
	S12: next_state = S15;
	S13: next_state = S15;
	S14: next_state = S15;
	S15: next_state = S0;
	S16: next_state = S0;
	S17: next_state = S0;
	S18: next_state = S0;
	S19: next_state = S20;
	S20: next_state = S21;
	S21: next_state = S0;
	S22: next_state = S23;
	S23: next_state = S0;
	default: next_state = 5'bxxxxx;
	endcase
end

endmodule

module ALUControl (ALUOp,Funct,ALUOpcode_MC);
output reg [2:0] ALUOp;
input [2:0] ALUOpcode_MC;
input [3:0] Funct;

always@(*)
begin
if(ALUOpcode_MC == 3'b100)
begin
case(Funct)
4'b0001 : ALUOp <= 3'b100;
4'b0010 : ALUOp <= 3'b101;
4'b0011 : ALUOp <= 3'b110;
default : ALUOp <= 3'bxxx;
endcase
end
else
ALUOp <= ALUOpcode_MC;
end

endmodule

module ALU_unit(A,B,ALUOp,out,zero);
input [15:0] A,B;
input [2:0] ALUOp;
output reg [15:0] out;
output zero;
integer i,count;

always @(*)
begin
	count = B;
	if(ALUOp==3'b000)
	begin
	out =A+B;
	end
	if(ALUOp==3'b001)
	begin
	out =A-B;
	end
	if(ALUOp==3'b010)
	begin
	out = ~(A & B);
	end
	if(ALUOp==3'b011)
	begin
	out =(A | B);
	end
	if(ALUOp==3'b100)
	begin
	out=A<<B;
	end
	if(ALUOp==3'b101)
	begin
	out=A>>B;
	end
	if(ALUOp==3'b110)
	begin
	out=A>>>B;
	end
end
assign zero = (out ==16'b0);
endmodule

module Register_File(ReadReg1,ReadReg2,ReadReg3,ReadReg4,ReadReg5,WriteReg,WriteData,ReadData1,ReadData2,ReadData3,ReadData4,ReadData5,RegWrite);
input [3:0] ReadReg1,ReadReg2,ReadReg3,ReadReg4,ReadReg5,WriteReg;
input [15:0] WriteData;
input RegWrite,clk;
output reg [15:0] ReadData1=0,ReadData2=0,ReadData3=0,ReadData4=0,ReadData5=0;
integer i;
reg [15:0] Registers [0:15] ;
initial begin
	for(i=0;i<16;i=i+1)
		Registers[i]=0;
end
always @ (*)
begin
Registers[0]=16'd0;
if(RegWrite==1'b0)
	begin
	ReadData1=Registers[ReadReg1];
	ReadData2=Registers[ReadReg2];
	ReadData3=Registers[ReadReg3];
	ReadData4=Registers[ReadReg4];
	ReadData5=Registers[ReadReg5];
	end
else
	begin
	Registers[WriteReg]<=WriteData;
	/*for(i=0;i<16;i=i+1)
		$display("%d",Registers[i]);*/
	end
end
endmodule

module Memory_1 (clk,MemRead,MemWrite,Address,WriteData,MemData);
input MemRead,MemWrite,clk;
input [15:0] WriteData;
input [15:0] Address;
output reg[15:0] MemData=0;
reg [7:0] storage[0:128];
initial begin
	$display("Loading ram data.");
	$readmemh("ram_data.dat", storage);
end
always @ (posedge clk)
begin
	if(MemWrite==1'b1 && clk==1'b1)
	begin
	storage[Address]=WriteData[7:0];
	storage[Address+1]=WriteData[15:8];
	$display("writing ram data.");
	$writememh("ram_data_wr.dat", storage);
	end
end
always @(MemRead)
begin
	if(MemRead==1'b1)
	begin
	MemData={storage[Address+1],storage[Address]};
	end
end
endmodule

module Memory_2 (MemRead,MemWrite,Address,WriteData,MemData);
input MemRead,MemWrite;
input [15:0] WriteData;
input [15:0] Address;
output reg[15:0] MemData;

reg [7:0] storage[0:128];
initial begin
	$display("Loading ram data.");
	$readmemh("ram_instr-1.dat", storage);
end
always @ (*)
begin
	if(MemRead==1'b1)
	begin
	MemData={storage[Address],storage[Address+1]};
	end
	else if(MemWrite==1'b1)
	begin
	storage[Address]=WriteData[7:0];
	storage[Address+1]=WriteData[15:8];
	end
end
endmodule


module Instruction_Register(clk,D,WriteEnable,m,n,x,y,z,b1,b2,b3,b4,b5,opcode,funct);
input [15:0] D;
input WriteEnable,clk;
reg [15:0] Instr;
output [3:0]m,n,x,y,z;
output [15:0]b1,b2,b3,b4,b5;
output [3:0] funct;
output [3:0] opcode;
always @(posedge clk)
begin
	if(WriteEnable==1'b1)
	begin
	Instr=D;
	end
end
assign x = Instr[3:0];
assign y = Instr[7:4];
assign z = Instr[11:8];
assign m = {2'b11,Instr[11:10]};
assign n = {2'b10,Instr[9:8]};
assign b1 = {Instr[7],Instr[7],Instr[7],Instr[7],Instr[7],Instr[7],Instr[7],Instr[7],Instr[7:0]};
assign funct = Instr[3:0];
assign opcode = Instr[15:12];
assign b5 = {Instr[7],Instr[7],Instr[7],Instr[7],Instr[7],Instr[7],Instr[7],Instr[7:0],1'b0};
assign b2 = {8'b0,Instr[7:0]};
assign b3 = {12'b0,Instr[7:4]};
assign b4 = {Instr[11],Instr[11],Instr[11],Instr[11],Instr[11:0]};

endmodule

module Program_Counter(CLK,PCSource,PCSrc0,PCSrc1,PCSrc2,PCWriteInput,PC,PCWrite);
input [1:0] PCSource;
input PCWrite;
input [15:0] PCSrc0,PCSrc1,PCSrc2;
input PCWriteInput,CLK;
output reg [15:0] PC=0;

always @(posedge CLK)
begin
	if(PCWriteInput==1'b1)
	begin
	case(PCSource)
	2'd0:PC=PCSrc0;
	2'd1:PC=PCSrc1;
	2'd2:PC=PCSrc2;
	endcase
	end
end
endmodule

module processor(clk, PC_out);
input clk;
wire [15:0] A,B,C,D,E;
wire PCWrite,PCWriteCond1,PCWriteCond2,InstrMemRead,MemRead,MemWrite,IRWrite,MemtoReg,RegWrite,RegDst,Zero;
wire [2:0]ALUSrcB,ALUOp;
wire [1:0]PCSource,ALUSrcA;
wire [15:0]ALUA,ALUB,ALUout,PC,InstrData,MDR;
wire [3:0]m,n,x,y,z;
wire [15:0]b1,b2,b3,b4,b5;
wire [3:0] op;
wire [3:0] func;
wire [2:0] ALUOp_MC;
wire IorD;
reg [15:0] ALUout_Reg,MDR_Reg;
output reg [15:0] PC_out=0;
control_unit Main_control (PCWrite,PCWriteCond1,PCWriteCond2,InstrMemRead,IorD,MemRead,MemWrite,IRWrite,MemtoReg,PCSource,ALUOp_MC,ALUSrcB,ALUSrcA,RegWrite,RegDst,clk,op,func,PC); //main control
ALUControl Alu_control (ALUOp,func,ALUOp_MC); // alu control
Memory_2 Instruction_Memory (InstrMemRead,1'b0,PC,16'd0,InstrData);  //instruction memory
Memory_1 Data_Memory (clk,MemRead,MemWrite,ALUout_Reg,E,MDR); //data memory
Instruction_Register IR (clk,InstrData,IRWrite,m,n,x,y,z,b1,b2,b3,b4,b5,op,func); //IR
ALU_unit ALU (ALUA,ALUB,ALUOp,ALUout,Zero); //ALU
Register_File RF (x,y,z,n,m,(RegDst ? m:z),(MemtoReg ? MDR_Reg : ALUout_Reg ),A,B,C,D,E,RegWrite); //Register File
Program_Counter PC_Register (clk,PCSource,ALUout_Reg,ALUout,C,(PCWrite || (PCWriteCond2 && !Zero) ||(PCWriteCond1 && Zero)),PC,PCWrite); //Program Counter

always @(posedge clk) begin
ALUout_Reg<=ALUout;
PC_out <= PC;
MDR_Reg<=MDR;
end

always@(ALUSrcB, ALUSrcA) begin
if(ALUSrcB==3'b000) begin
$display("******New Instruction*******");
$display("IF stage : %d", PC);
end
if(ALUSrcB==3'b101)
$display("ID stage. Instruction decoded with - %b", op);
if(MemtoReg==1'b1)
$display("WB stage: MDR is written with Reg['11'+IR[11:10]]");
end

always@(MemtoReg) begin
if(MemtoReg==1'b1)
$display("WB stage: MDR is written with Reg['11'+IR[11:10]]");
end

assign ALUB = ALUSrcB[2] ? (ALUSrcB[1] ? b5 :(ALUSrcB[0] ? b4 : b3 )):(ALUSrcB[1] ? (ALUSrcB[0] ? b2 : b1):(ALUSrcB[0] ? A : 16'd2) );
assign ALUA = ALUSrcA[1] ? (ALUSrcA[0] ? D:C):(ALUSrcA[0] ? B:PC);

endmodule


module tb_multi_cycle;
  wire [15:0] out;
  reg CLK;
  always #10 CLK = ~CLK;

  processor p1(.clk(CLK), .PC_out(out));

  initial begin
	$dumpfile("final.vcd");
    $dumpvars(0,tb_multi_cycle);
    CLK=1'b1;
    #2600 $finish;
  end

endmodule
