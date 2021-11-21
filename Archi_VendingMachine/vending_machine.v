// Title         : vending_machine.v
// Author      : Jae-Eon Jo (Jojaeeon@postech.ac.kr) 
//					   Dongup Kwon (nankdu7@postech.ac.kr) (2015.03.30)

`include "vending_machine_def.v"

module vending_machine (

	clk,							// Clock signal
	reset_n,						// Reset signal (active-low)
	
	i_input_coin,				// coin is inserted.
	i_select_item,				// item is selected.
	i_trigger_return,			// change-return is triggered 
	
	o_available_item,			// Sign of the item availability
	o_output_item,			// Sign of the item withdrawal
	o_return_coin				// Sign of the coin return
);

	// Ports Declaration
	// Do not modify the module interface
	input clk;
	input reset_n;
	
	input [`kNumCoins-1:0] i_input_coin;
	input [`kNumItems-1:0] i_select_item;
	input i_trigger_return;
		
	output [`kNumItems-1:0] o_available_item;
	output [`kNumItems-1:0] o_output_item;
	output [`kNumCoins-1:0] o_return_coin;
 
	// Normally, every output is register,
	//   so that it can provide stable value to the outside.
	reg [`kNumItems-1:0] o_available_item;
	reg [`kNumItems-1:0] o_output_item;
	reg [`kNumCoins-1:0] o_return_coin;
	
	// Net constant values (prefix kk & CamelCase)
	// Please refer the wikepedia webpate to know the CamelCase practive of writing.
	// http://en.wikipedia.org/wiki/CamelCase
	// Do not modify the values.
	wire [31:0] kkItemPrice [`kNumItems-1:0];	// Price of each item
	wire [31:0] kkCoinValue [`kNumCoins-1:0];	// Value of each coin
	assign kkItemPrice[0] = 400;
	assign kkItemPrice[1] = 500;
	assign kkItemPrice[2] = 1000;
	assign kkItemPrice[3] = 2000;
	assign kkCoinValue[0] = 100;
	assign kkCoinValue[1] = 500;
	assign kkCoinValue[2] = 1000;


	// NOTE: integer will never be used other than special usages.
	// Only used for loop iteration.
	// You may add more integer variables for loop iteration.
	integer i, j, k;

	// Internal states. You may add your own net & reg variables.
	reg [`kTotalBits-1:0] current_total;
	
	// Next internal states. You may add your own net and reg variables.
	reg [`kTotalBits-1:0] current_total_nxt;
	
	// Variables. You may add more your own registers.
	reg [`kTotalBits-1:0] input_total, output_total, return_total;
	reg [31:0] waitTime;
	parameter first = 3'b000, coin = 3'b001, stay = 3'b010, print = 3'b011, change = 3'b100;
	reg [2:0] CS;
	reg [2:0] NS;
	reg [`kNumCoins-1:0] input_coin;
	reg [`kNumItems-1:0] select_item;
	reg [1:0] output_item;
	reg [1:0] return_coin;
	reg trigger_return;

	// initiate values
	initial begin
		// TODO: initiate values
		current_total = 'd0;
		current_total_nxt = 'd0;
		input_total = 'd0;
		output_total = 'd0;
		return_total = 'd0;
		waitTime = 'd0;
		CS = first;
		NS = first;
	end

	always @(i_input_coin) begin
		input_coin = i_input_coin;
	end
	
	always @(i_select_item) begin
		select_item = i_select_item;
	end

	always @(i_trigger_return) begin
		if(i_trigger_return == 'b1)
			trigger_return = 'b1;
	end
	
	// Combinational logic for the next states
	always @(CS, input_coin, select_item, waitTime) begin
		// TODO: current_total_nxt
		// You don't have to worry about concurrent activations in each input vector (or array).
		case(CS)
			first: begin
				if(input_coin)
					NS = coin;
				else
					NS = first;
			end
			coin: begin
				for(i = 0; i < `kNumCoins; i = i+1)
					if(i_input_coin[i]) begin
						current_total_nxt = current_total_nxt + kkCoinValue[i];
						input_coin[i] = 0;
					end
				waitTime = `kWaitTime;
				NS = stay;
			end
			stay: begin
				if(input_coin)
					NS = coin;
				if(select_item)
					NS = print;
				if(waitTime == 0)
					NS = change;
			end
			print: begin
				for(i = 0; i < `kNumItems; i = i+1)
					if(select_item[i])
						if(current_total >= kkItemPrice[i]) begin
							current_total_nxt = current_total_nxt - kkItemPrice[i];
							output_item = i;

							select_item[i] = 0;
							waitTime = `kWaitTime;
						end
				NS = stay;
			end
			change: begin
				if(current_total_nxt == 0)
					NS = first;
				else begin
					NS = change;
					for(i = `kNumCoins; i >= 0; i = i-1)
						if(kkCoinValue[i] < current_total_nxt) begin
							current_total_nxt = current_total_nxt - kkCoinValue[i];
							return_coin = i;
						end
				end
			end
		endcase
		// Calculate the next current_total state.
		
		// You may add more next states.
		
	end

	
	
	// Combinational logic for the outputs
	always @(current_total, output_item, return_coin) begin
		// TODO: o_available_item
		for(i = 0; i < `kNumItems; i = i+1) begin
			if(current_total >= kkItemPrice[i])
				o_available_item[i] = 1'b1;
			else
				o_available_item[i] = 1'b0;
		end
		// TODO: o_output_item
		for(i = 0; i < `kNumItems; i = i+1) begin
			if(i == output_item)
				o_output_item[i] = 1'b1;
			else
				o_output_item[i] = 1'b0;
		end
		// TODO: o_return_coin
		for(i = 0; i < `kNumCoins; i = i+1) begin
			if(i == return_coin)
				o_return_coin[i] = 1'b1;
			else
				o_return_coin[i] = 1'b0;
		end
	end
 
	
	
	// Sequential circuit to reset or update the states
	always @(posedge clk) begin
		//$fdisplay(f, "%d, %d", current_total, current_total_nxt);
		if (!reset_n) begin
			// TODO: reset all states.
			current_total = 'd0;
			current_total_nxt = 'd0;
			input_total = 'd0;
			output_total = 'd0;
			return_total = 'd0;
			waitTime = 'd0;
			CS = first;
			NS = first;
		end
		else begin
			// TODO: update all states.
			current_total = current_total_nxt;
			waitTime = waitTime - 1;
			if (trigger_return == 1)
				NS = change;
			CS = NS;
		end
	end

endmodule

