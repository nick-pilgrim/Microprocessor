//Nicholas Pilgrim-Minaya
//ASU ID 1215099860 

//This file contains all of the modules necessary to run the microprocessor

integer fd; //Integer for excel file log
module micro(input clk, reset, output logic [3:0] OPCODE, output logic [1:0] state, output logic [7:0] PC,
	alu_out, W_Reg, output logic Cout, OF); //This is the main module
localparam IF=2'b00, FD=2'b01, EX=2'b10, RWB=2'b11; //State declaration
logic [15:0] IR; //Instruction register
logic [1:0] next_state; //next state handler
logic [7:0] next_PC; //future pc handler
logic [3:0] RA, RB, RD; 
logic [7:0] D, A, B, Bn,A_,B_; //contents of RA RB RD
logic Clast; //For adder function
typedef struct{ logic S; logic Cout_; } FA1bsol; //for 1 bit adder function
typedef struct{ logic [7:0] Sn_; logic Cnout_; logic OFn_;} FAnbsol; //for adder functio
FAnbsol Dn, two_comp;
ROM_micro Instruction_Memory(PC, IR); //instantation of ROM
RegFile_micro RL5(reset, clk, RA, RB, RD, OPCODE, state, D, A_, B_); //Instantiation of Register file
W_Register W_R(W_Reg, clk, reset, alu_out); //W register instantation
always_ff @(posedge clk or posedge reset) begin
	if(reset) begin
		fd = $fopen("log_file_micro.csv");
		$fwrite(fd, "PC, IR, OPCODE, RA, RB, RD, W_Reg, Cout, OF\n"); LOG FILE LINE COMMENTED OUT FOR QUARTUS
		state <= IF; 
		PC <= 8'd0;
	end
	else begin
		state <= next_state;
		PC <= next_PC;
	end
end
always_comb begin
next_state = state; //store variables to prevent latches
D = W_Reg;
next_PC = PC;
RA = IR[11:8];
RB = IR[7:4];
RD = IR[3:0];
OPCODE = IR[15:12];
A = A_;
B = B_; 
Cout = 1'b0; 
OF = 1'b0; 
alu_out=W_Reg;
Bn = twocomp(B);
	case(state) //case statement to handle state diagram and functions
		IF: begin //this state loads IR into RA, RB, RD, and opcode
			next_state = FD; 
			next_PC = PC;
			OPCODE = IR[15:12];
			RA = IR[11:8];
			RB = IR[7:4];
			RD = IR[3:0]; end
		FD: begin
			next_state = EX; //extracts A from RA
			next_PC = PC;
			A = A_;
			B = B_; end
		EX: begin
			next_state = RWB; 
			next_PC = PC; 
			alu_out=8'd0;Bn = 8'b0; Cout = 1'b0; OF = 1'b0;
			case(OPCODE) //***ALU*** 
				4'b0001: begin //add
					Dn = FAn_micro(A,B, 'b0);
					alu_out = Dn.Sn_;
					Cout = Dn.Cnout_;
					OF = Dn.OFn_; end //ADD
				4'b0010: begin alu_out= {RA, RB}; end //LDI
				4'b0011: begin //SUB
					//$display("Two's Complement input: B=%h", B);
					Bn = twocomp(B);
					//$display("Two's Complement output: Bn=%h", Bn);
					Dn = FAn_micro(A,Bn, 'b0);
					alu_out = Dn.Sn_;
					Cout = Dn.Cnout_;
					OF = 1'b0; end //sub
				4'b0100: begin //ADI
					//$display("OPCODE 4: Inputs into the Adder Function: A=%h, B=%h", A, B);  
					Dn = FAn_micro(A,{4'd0, RB}, 'b0);
					alu_out = Dn.Sn_;	
					//$display("Full Adder solution: alu_out=%h", alu_out);  
					Cout = Dn.Cnout_;
					OF = Dn.OFn_; end //ADI
				4'b0101: begin alu_out = div_(A,B); end //div
				4'b0110: begin alu_out = A*B; end // mul
				4'b0111: begin alu_out = B- 8'd1; end //DEC
				4'b1000: begin //alu_out = B + 8'd1; 
					Dn = FAn_micro(B,8'd1, 'b0);
					alu_out = Dn.Sn_;
					Cout = Dn.Cnout_;
					OF = Dn.OFn_; end //INC
				4'b1001: begin alu_out = ~(A|B); end //NOR
				4'b1010: begin alu_out = ~(A&B); end //NAND
				4'b1011: begin alu_out = A^B; end //XOR
				4'b1100: begin alu_out = ~B; end //comp
				//4'b1101: begin end //cjmp
				//4'b1110: begin end //jmp
				//4'b1111: begin end //HLT
				default: begin alu_out=8'd0; Bn = 8'b0; Cout = 1'b0; OF = 1'b0;end
			endcase
		end
		RWB: begin next_state = IF;
			next_PC = PC + 8'd1;
			D = W_Reg;
			$fwrite(fd,"%h, %h, %h, %h, %h, %h, %h, %h, %h\n", PC, IR, OPCODE, RA, RB, RD, W_Reg, Cout, OF);
			if((OPCODE == 4'b1101) && (A >= B)) next_PC = PC + RD; //CJMP
			if(OPCODE == 4'b1110) next_PC = {RA,RB}; //JMP
			if(OPCODE == 4'b1111) next_PC = PC; end //HALT
		default: begin next_state=state; next_PC = PC; //Preventing latches
				RA = IR[11:8];
				RB = IR[7:4];
				RD = IR[3:0];
				OPCODE = IR[15:12];
				D = W_Reg;
				A = A_;
				B = B_; 
				Bn = twocomp(B);
				Cout = 1'b0; 
				OF = 1'b0; 
				alu_out=W_Reg;end
	endcase	
		
end
function FA1bsol FA_micro; //1 bit full adder
	input A,B,Cin;
	reg C, Cout;
	FA_micro.S = A^B^Cin;
	FA_micro.Cout_ = A&B|A&Cin|B&Cin;
endfunction

function FAnbsol FAn_micro; //n bit full adder
	input [7:0] A, B;
	input Cin;
	FA1bsol E;
	integer i;
	FAn_micro.Cnout_ = Cin;
	//$display("INSIDE FULL ADDER FUNCTION: A=%h, B=%h", A, B);  
	for(i=0; i<=7; i=i+1) begin
		Clast=FAn_micro.Cnout_;
		E=FA_micro(A[i], B[i], FAn_micro.Cnout_);
		FAn_micro.Sn_[i]= E.S;
		FAn_micro.Cnout_=E.Cout_;
	end
	FAn_micro.OFn_ = FAn_micro.Cnout_^Clast;
endfunction

function [7:0] twocomp;
	input [7:0] A;
	reg [7:0] An;
	//$display("Inside Two's Complement Function A=%h", A);
	An = ~A;
	two_comp = FAn_micro(An, 8'd0, 'b1);
	twocomp = two_comp.Sn_;
endfunction
/*
function [7:0] mul;
	input [7:0] A,B;
	$display("A=%h, B=%h",A,B);
	reg [7:0] C;
	integer i;
	for(i=1; i<8; i=i+1) begin
		if(B[i] != 0)
			mul = mul + A<<i;
	if(B[0] !=0) mul = mul + A; end
endfunction
*/
function [7:0] div_; //divider module
	input [7:0] A, B;
	//$display("A=%h, B=%h", A, B);
	reg [7:0] A_;
	div_ = {8{1'b0}};
	A_ = A;
	if(A_ > B)
		begin: repeat_block
		repeat({8{1'b1}}) begin
			A_=A_-B;
			div_=div_ + {{(7){1'b0}},1'b1};
			//$display("A_=%h", A_, B);
			if(A_ < B) begin
				$display("disable repeat_block");
				disable repeat_block;
			end
		end
	end
endfunction

endmodule

module W_Register(output reg [7:0] Q, input clk, Reset, input [7:0] D); //D flip flop for w register
always_ff@ (posedge clk or posedge Reset)
	if(Reset)
		Q <= 8'd0;
	else
		Q <= D;
endmodule



//TEST BENCH for the the simulation verification
module micro_tb();
logic [3:0] OPCODE; //output
logic [1:0] State; //output
logic [7:0] PC, alu_out, W_Reg; //output
logic clk, reset,Cout, OF;
integer i;
micro l5(clk, reset,OPCODE, State, PC, alu_out, W_Reg, Cout, OF); //instantiation of micro module
initial begin
	i=0; clk=0; reset=1; #5;
	repeat(360) begin //run until the end so that it pauses at halt
		clk = 0; reset=0; #5;
		i=i+1;
		$display("i=%d, clk=%b, State=%b, alu_out=%h, OPCODE=%h, W_Reg=%h, Cout=%h, OF=%h", i,clk, State, alu_out, OPCODE, W_Reg, Cout, OF);
		clk = 1; #5;
	end
	$fclose(fd);
end
endmodule

//THIS IS THE MEMORY MODULE 
module ROM_micro(input [7:0] PC, output logic [15:0] IR);
logic [15:0] mem [20:0];
assign mem[0] = 16'h2000; //USING HEX NUMBERS
assign mem[1] = 16'h2011;
assign mem[2] = 16'h2002;
assign mem[3] = 16'h20A3;
assign mem[4] = 16'hD236;
assign mem[5] = 16'h1014;
assign mem[6] = 16'h4100;
assign mem[7] = 16'h4401;
assign mem[8] = 16'h8022;
assign mem[9] = 16'hE040;
assign mem[10] = 16'h4405;
assign mem[11] = 16'h5536;
assign mem[12] = 16'h6637;
assign mem[13] = 16'h3538;
assign mem[14] = 16'h4329;
assign mem[15] = 16'h709A;
assign mem[16] = 16'h70AB;
assign mem[17] = 16'hBB8C;
assign mem[18] = 16'h9D8E;
assign mem[19] = 16'hC0EF;
assign mem[20] = 16'hF000;
assign IR = mem[PC];
endmodule

//THIS IS THE REGISTER FILE MODULE 
module RegFile_micro(input reset, clk, input [3:0] RA, input [3:0] RB, input [3:0] RD,
	input [3:0] OPCODE, input [1:0] current_state, input [7:0] RF_data_in,
	output logic [7:0] RF_data_out0, output logic [7:0] RF_data_out1);
localparam IF = 2'b00, FD = 2'b01, EX = 2'b10, RWB = 2'b11; //STATE ASSIGNMENTS
logic [7:0] RF [15:0]; //RF FILE
always_ff @(posedge clk or posedge reset)begin
	if(reset) begin
		RF_data_out0 = 8'd0; //RESET OUTPUTS TO ZERO
		RF_data_out1 = 8'd0;
		RF[0] = 8'd0; RF[1] = 8'd0; RF[2] = 8'd0; RF[3] = 8'd0; //RESETS THE REGISTER file
		RF[4] = 8'd0; RF[5] = 8'd0; RF[6] = 8'd0; RF[7] = 8'd0;	
		RF[8] = 8'd0; RF[9] = 8'd0; RF[10] = 8'd0; RF[11] = 8'd0;
		RF[12] = 8'd0; RF[13] = 8'd0; RF[14] = 8'd0; RF[15] = 8'd0;
	end
	else begin
		RF_data_out0 <= RF[RA]; 
		RF_data_out1 <= RF[RB];
		//$display("A=%h, RA=%h, B=%h, RB= %h", RF[RA], RA, RF[RB], RB);
		//WRITE TO ALU OUT ON REGISTER WRITE BACK CYCLE
		//DONT WRITE WHEN OPCODE REPRESENTS JMP HLT CMPJ
		if(~((OPCODE == 4'b1110)||(OPCODE == 4'b1101)) && (current_state == RWB) )begin
			RF[RD] <= RF_data_in; $display("D=%h, RD=%h", RF_data_in, RD); end
			
	end
end
endmodule

module pmcntr_micro #(parameter size = 5) (input clk, reset, input [size-1:0] count_max, //counter to make 50M hz equal 2 hz
	output logic [size-1:0] count, output logic clkout);
always_ff @ (posedge clk or posedge reset) begin
	if(reset) begin
		count <= {size{1'b0}};
		clkout <= 1'b0;
	end
	else if (count < count_max) begin
		count <= count + {{(size-1){1'b0}}, 1'b1};
	end
	else begin
		count <= {size{1'b0}};
		clkout <= ~clkout;
	end
end
endmodule

module micro_pv(input clk, SW0, SW1, KEY0, SW2, SW3,SW4,
	output logic [6:0] SEG5, SEG4, SEG3, SEG2, SEG1, SEG0,
	output logic LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7);
logic s0, s1, s2, s3, s4, clk_, clk_2Hz, clk_2Hz_, Cout, OF;
logic [3:0] OPCODE, PC_1,PC_2, Wreg_1, Wreg_2, alu_0, alu_1, OPCODE_;
logic [1:0] state;
logic [2:0] switches;
logic [7:0] PC, alu_out, W_Reg;
logic [6:0] PC1,PC2, Wreg1,Wreg2, alu0, alu1, _OPCODE;
logic [22:0] count;
pmcntr_micro #(23) cntr_micro(clk_, s1, 23'd2500000, count, clk_2Hz_);
micro parent(clk_2Hz, s0, OPCODE, state, PC, alu_out, W_Reg, Cout, OF);
HEX_micro h0(PC_1, PC1);
HEX_micro h1(PC_2, PC2);
HEX_micro h2(Wreg_1, Wreg1);
HEX_micro h3(Wreg_2, Wreg2);
HEX_micro h4(alu_0, alu0);
HEX_micro h5(alu_1, alu1);
HEX_micro h6(OPCODE_, _OPCODE);
always_comb begin
	LED0=PC[0]; LED1=PC[1]; LED2=PC[2]; LED3=PC[3]; LED4=PC[4]; LED5=PC[5]; LED6=PC[6]; LED7=PC[7];
	PC_1 = PC[3:0];
	PC_2 = PC[7:4];
	Wreg_1= W_Reg[3:0];
	Wreg_2= W_Reg[7:4];
	alu_0 = alu_out[3:0];
	alu_1 = alu_out[7:4];
	OPCODE_ = OPCODE;
	s0 = SW0; //reset
	clk_ = clk; // variable connections
	clk_2Hz =1'b0;
	switches = {SW4,SW3,SW2};
	SEG0 = 7'b1111111;
	SEG1 = 7'b1111111;
	SEG2 = 7'b1111111;
	SEG3 = 7'b1111111; 
	SEG4 = 7'b1111111; 
	SEG5 = 7'b1111111; 
	if((SW1 == 1'b0) && (OPCODE != 4'b1111)) clk_2Hz = clk_2Hz_;
	if((KEY0==1'b0) && (SW1 == 1'b1)) begin clk_2Hz = ~clk_2Hz_; clk_2Hz = ~clk_2Hz_; end //clk_2Hz = ~clk_2Hz_; end// if switch one is up and key is pressed, the alarm counts DOESNT WORK
	case(switches)
		3'b000: begin
			SEG5 = 7'b0001100; //P
			SEG4 = 7'b1001111; //I
			SEG3 = 7'b1000111; //L
			SEG2 = 7'b0010000; //G
			SEG1 = 7'b0101111; //R
			SEG0 = 7'b1001111; //I
		end
		3'b110: begin
			SEG5 = 7'b0001100; //P
			SEG4 = 7'b1000110; //C
			SEG3 = 7'b1111111; //0
			SEG2 = 7'b1111111; //0
			SEG1 = PC2; //PC[7:4]
			SEG0 = PC1; //PC[3:0] 
		end
		3'b101: begin
			SEG5 = 7'b1010101; // W
			SEG4 = 7'b1110111; //_
			SEG3 = 7'b0101111; // r
			SEG2 = 7'b1111111; //0
			SEG1 = Wreg2; //W_Reg[7:4]
			SEG0 = Wreg1; //W_Reg[3:0]
		end
		3'b011: begin
			SEG5 = 7'b0001000; //A
			SEG4 = 7'b1000111; //L
			SEG3 = 7'b1000001; //U
			SEG2 = 7'b1111111; //0
			SEG1 = alu1; //alu[7:4]
			SEG0 = alu0; //alu[3:0] //0
		end
		3'b111: begin
			SEG5 = 7'b1000000; // O
			SEG4 = 7'b0001100; // P
			SEG3 = 7'b1000110; // c
			SEG2 = 7'b1111111; //0
			SEG1 = 7'b1111111; //0
			SEG0 = _OPCODE; //OPCODE
		end
		default: begin 	SEG0 = 7'b1111111;
				SEG1 = 7'b1111111;
				SEG2 = 7'b1111111;
				SEG3 = 7'b1111111; 
				SEG4 = 7'b1111111; 
				SEG5 = 7'b1111111; end
	endcase

end
endmodule


module HEX_micro(input [4:0] hex, output reg [6:0] HexSeg);
	always_comb begin
		HexSeg = 7'd0;
		case(hex)
			4'h0 : HexSeg = 7'b1000000;
			4'h1 : HexSeg = 7'b1111001;
			4'h2 : HexSeg = 7'b0100100; // 2
			4'h3 : HexSeg = 7'b0110000; // 3
			4'h4 : HexSeg = 7'b0011001; // 4
			4'h5 : HexSeg = 7'b0010010; // 5
			4'h6 : HexSeg = 7'b0000010; // 6
			4'h7 : HexSeg = 7'b1111000; // 7
			4'h8 : HexSeg = 7'b0000000; // 8
			4'h9 : HexSeg = 7'b0010000; // 9
			4'hA : HexSeg = 7'b0001000; //A
			4'hB : HexSeg = 7'b0000011; // B
			4'hC : HexSeg = 7'b1000110; // C 1000110
			4'hD : HexSeg = 7'b0100001; // D 0100001
			4'hE : HexSeg = 7'b0000110; // E
			4'hF : HexSeg = 7'b0001110; // F
			default: begin HexSeg = 7'd0; end
		endcase
	end
endmodule

