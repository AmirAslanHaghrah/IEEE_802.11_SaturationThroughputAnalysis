// <IEEE_802.11_SaturationThroughputAnalysis>
//
// Copyright © <2018> <AmirAslan Haghrah>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files(the "Software"), to deal in
// the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <iostream>
#include <vector>
#include <algorithm>

#define EPSILON 0.00000001

double collisionProbability(double p, int W, int m, int n);
double rootOfCollisionProbability(int W, int m, int n);

void main() {
	// Numerical calculations based on Bianchis article
	// Set default parameters from the article
	int SIFS = 28;				// us
	int DIFS = 128;				// us
	int slot_time = 50;			// us
	int prop_delay = 1;			// us

	int PayLoad = 8184;			// bits
	int MAC_header = 272;		// bits
	int PHY_header = 128;		// bits
	int ack = 112;				// bits

	int RTS = 160 + PHY_header; //bits
	int CTS = 112 + PHY_header; //bits

	int Packet = MAC_header + PHY_header + PayLoad;			// bits
	int ACK = 112 + PHY_header;								// bits

	double Ts_Basic = Packet + SIFS + prop_delay + ACK + DIFS + prop_delay;
	double Tc_Basic = Packet + DIFS + prop_delay;

	double Ts_RTS = RTS + SIFS + prop_delay + CTS + SIFS + prop_delay + Packet + SIFS + prop_delay + ACK + DIFS + prop_delay;
	double Tc_RTS = RTS + DIFS + prop_delay;

	// Set the window size and stage number by typing inputs
	int W = 32;
	int m = 3;
	std::cout << "Minimum backoff window size: (in slot time)\n";
	std::cin >> W;
	std::cout << "Maximum stage number: \n";
	std::cin >> m;

	// Computation Bianchi Throughput Saturation Basic Model
	std::cout << "Computation Bianchi Throughput Saturation Basic Model...\n";
	double ComputationalBasicModelThroughput[50] = {};

	for (int n = 5; n <= 50; n++) {
		// P is the probability that transmitted packet collide
		double P = rootOfCollisionProbability(W, m, n);
		// tau is probability that a station transmits in a generic slot time
		double tau = 2.0f * (1.0f - 2.0f * P) / ((1.0f - 2.0f * P) * (W + 1.0f) + P * W *(1.0f - std::pow((2.0f * P), m)));
		// Ptr is that in a slot time there is at least one transmission
		double Ptr = 1.0f - std::pow((1.0f - tau), n);
		// Ps is the probability that a transmission is successful
		double Ps = n * tau * (std::pow((1.0f - tau), (n - 1))) / Ptr;	
		ComputationalBasicModelThroughput[n - 5] = Ps * Ptr * PayLoad / ((1 - Ptr) * slot_time + Ptr * Ps * Ts_Basic + Ptr * (1.0f - Ps) * Tc_Basic);
	}
	std::cout << "\n\tDone\n\n";
	
	// Simulation
	std::cout << "Simulating Bianchi Throughput Saturation Basic Model...\n";
	long Total_Simulation_Time = 0;
	double SimulationBasicModelThroughput[50] = {};
	std::vector<long> Station_Stage;
	std::vector<long> Station_Backoff_Time;

	for (int station = 5; station <= 50; station++) {
		long Successful_Packet_Transmit_Count = 0;
		long Collided_Packet_Count = 0;
		Station_Stage.resize(station, 0);
		Station_Backoff_Time.resize(station, 0);

		// initial backoff time
		for (int i = 0; i < station; i++) {
			Station_Stage[i] = 0;
			Station_Backoff_Time[i] = DIFS + std::floor(W * std::rand() / RAND_MAX) * slot_time;
		}

		while (Successful_Packet_Transmit_Count < 100000) {
			long Min_Station_Backoff_Time = *std::min_element(Station_Backoff_Time.begin(), Station_Backoff_Time.end());
			int Simultaneous_Tranmit_Count = 0;
			for (int i = 0; i < Station_Backoff_Time.size(); i++) {
				if (Min_Station_Backoff_Time == Station_Backoff_Time[i]) {
					Simultaneous_Tranmit_Count++;
				}
			}

			// Sucessful reansition
			if (Simultaneous_Tranmit_Count == 1) {
				Successful_Packet_Transmit_Count++;
				for (int i = 0; i < station; i++) {
					// uniform backoff at stage 0
					if (Station_Backoff_Time[i] == Min_Station_Backoff_Time) {
						if (Successful_Packet_Transmit_Count == 100000) {
							Total_Simulation_Time = Station_Backoff_Time[i] + Ts_Basic;
						}
						Station_Stage[i] = 0;
						Station_Backoff_Time[i] = Station_Backoff_Time[i] + Ts_Basic + floor(W * std::rand() / RAND_MAX) * slot_time;
					}
					// stop counting while the channel is busy
					else {
						Station_Backoff_Time[i] = Station_Backoff_Time[i] + Ts_Basic;
					}
				}
			}
			else {
				Collided_Packet_Count++;
				for (int i = 0; i < station; i++) {
					// uniform backoff at next stage
					if (Station_Backoff_Time[i] == Min_Station_Backoff_Time) {
						if (Station_Stage[i] < m) {
							Station_Stage[i]++;
						}
						Station_Backoff_Time[i] = Station_Backoff_Time[i] + Tc_Basic + floor(std::pow(2.0f, Station_Stage[i]) * W * std::rand() / RAND_MAX) * slot_time;
					}
					else {
						Station_Backoff_Time[i] = Station_Backoff_Time[i] + Tc_Basic;
					}
				}
			}
		}
		SimulationBasicModelThroughput[station - 5] = (double)Successful_Packet_Transmit_Count * PayLoad / Total_Simulation_Time;
	}
	std::cout << "\tDone\n\n";



	// Computation Bianchi Throughput Saturation Basic Model
	std::cout << "Computation Bianchi Throughput Saturation RTS/CTS Model...\n";
	double ComputationalRTSModelThroughput[50] = {};

	for (int n = 5; n <= 50; n++) {
		// P is the probability that transmitted packet collide
		double P = rootOfCollisionProbability(W, m, n);
		// tau is probability that a station transmits in a generic slot time
		double tau = 2.0f * (1.0f - 2.0f * P) / ((1.0f - 2.0f * P) * (W + 1.0f) + P * W *(1.0f - std::pow((2.0f * P), m)));
		// Ptr is that in a slot time there is at least one transmission
		double Ptr = 1.0f - std::pow((1.0f - tau), n);
		// Ps is the probability that a transmission is successful
		double Ps = n * tau * (std::pow((1.0f - tau), (n - 1))) / Ptr;
		ComputationalRTSModelThroughput[n - 5] = Ps * Ptr * PayLoad / ((1 - Ptr) * slot_time + Ptr * Ps * Ts_RTS + Ptr * (1.0f - Ps) * Tc_RTS);
	}
	std::cout << "\n\tDone\n\n";


	// Simulation
	std::cout << "Simulating Bianchi Throughput Saturation RTS/CTS Model...\n";
	long Total_RTS_Simulation_Time = 0;
	double SimulationRTSModelThroughput[50] = {};

	for (int station = 5; station <= 50; station++) {
		long Successful_Packet_Transmit_Count = 0;
		long Collided_Packet_Count = 0;
		Station_Stage.resize(station, 0);
		Station_Backoff_Time.resize(station, 0);

		// initial backoff time
		for (int i = 0; i < station; i++) {
			Station_Stage[i] = 0;
			Station_Backoff_Time[i] = DIFS + std::floor(W * std::rand() / RAND_MAX) * slot_time;
		}

		while (Successful_Packet_Transmit_Count < 100000) {
			long Min_Station_Backoff_Time = *std::min_element(Station_Backoff_Time.begin(), Station_Backoff_Time.end());
			int Simultaneous_Tranmit_Count = 0;
			for (int i = 0; i < Station_Backoff_Time.size(); i++) {
				if (Min_Station_Backoff_Time == Station_Backoff_Time[i]) {
					Simultaneous_Tranmit_Count++;
				}
			}

			// Sucessful reansition
			if (Simultaneous_Tranmit_Count == 1) {
				Successful_Packet_Transmit_Count++;
				for (int i = 0; i < station; i++) {
					// uniform backoff at stage 0
					if (Station_Backoff_Time[i] == Min_Station_Backoff_Time) {
						if (Successful_Packet_Transmit_Count == 100000) {
							Total_RTS_Simulation_Time = Station_Backoff_Time[i] + Ts_RTS;
						}
						Station_Stage[i] = 0;
						Station_Backoff_Time[i] = Station_Backoff_Time[i] + Ts_RTS + floor(W * std::rand() / RAND_MAX) * slot_time;
					}
					// stop counting while the channel is busy
					else {
						Station_Backoff_Time[i] = Station_Backoff_Time[i] + Ts_RTS;
					}
				}
			}
			else {
				Collided_Packet_Count++;
				for (int i = 0; i < station; i++) {
					// uniform backoff at next stage
					if (Station_Backoff_Time[i] == Min_Station_Backoff_Time) {
						if (Station_Stage[i] < m) {
							Station_Stage[i]++;
						}
						Station_Backoff_Time[i] = Station_Backoff_Time[i] + Tc_RTS + floor(std::pow(2.0f, Station_Stage[i]) * W * std::rand() / RAND_MAX) * slot_time;
					}
					else {
						Station_Backoff_Time[i] = Station_Backoff_Time[i] + Tc_RTS;
					}
				}
			}
		}
		SimulationRTSModelThroughput[station - 5] = (double)Successful_Packet_Transmit_Count * PayLoad / Total_RTS_Simulation_Time;
	}
	std::cout << "\tDone\n\n";


	//Plot the results
	std::vector<float> x;
	std::vector<float> y1, y2, y3, y4;

	for (int i = 5; i <= 50; i++) {
		x.push_back(i);
		y1.push_back(ComputationalBasicModelThroughput[i - 5]);
		y2.push_back(SimulationBasicModelThroughput[i - 5]);
		y3.push_back(ComputationalRTSModelThroughput[i - 5]);
		y4.push_back(SimulationRTSModelThroughput[i - 5]);
	}

	FILE * gp = _popen("gnuplot", "w");
	fprintf(gp, "set terminal wxt size 600,400 \n");
	fprintf(gp, "set grid \n");
	fprintf(gp, "set title '%s' \n", "Bianchi Model IEEE 802.11 Saturation Throughput Analysis");
	fprintf(gp, "set style line 1 lt 3 pt 7 ps 0.3 lc rgb 'blue' lw 2 \n");
	fprintf(gp, "set style line 2 lt 3 pt 7 ps 0.3 lc rgb 'red' lw 1 \n");
	fprintf(gp, "set style line 3 lt 4 pt 7 ps 0.3 lc rgb 'green' lw 2 \n");
	fprintf(gp, "set style line 4 lt 5 pt 7 ps 0.3 lc rgb 'orange' lw 1 \n");
	fprintf(gp, "plot '-' w p ls 1 title 'Basic Theory', '-' w p ls 2 title 'Basic Simulation', '-' w p ls 3 title 'RTS Theory', '-' w p ls 4 title 'RTS Simulation'\n");

	for (int k = 0; k < x.size(); k++) {
		fprintf(gp, "%f %f \n", x[k], y1[k]);
	}
	fprintf(gp, "e\n");

	for (int k = 0; k < x.size(); k++) {
		fprintf(gp, "%f %f \n", x[k], y2[k]);
	}
	fprintf(gp, "e\n");

	for (int k = 0; k < x.size(); k++) {
		fprintf(gp, "%f %f \n", x[k], y3[k]);
	}
	fprintf(gp, "e\n");

	for (int k = 0; k < x.size(); k++) {
		fprintf(gp, "%f %f \n", x[k], y4[k]);
	}
	fprintf(gp, "e\n");

	fflush(gp);

	system("pause");
	_pclose(gp);
}

/// <summary>Collision Probability Calculation Function</summary>
/// <param name="p">Each packet collides with constant and independent probability p</param>  
/// <param name="W">Minimum backoff window size</param> 
/// <param name="m">Maximum stage number</param> 
/// <param name="n">Maximum stations number</param> 
/// <returns> </returns>  
double collisionProbability(double p, int W, int m, int n) {
	return (p - 1.0f + std::pow((1.0f - 2.0f * (1.0f - 2.0f * p) / ((1.0f - 2.0f * p)*(W + 1.0f) + p * W * (1.0f - std::pow((2.0f * p), m)))), (n - 1.0f)));
}

/// <summary>Collision Probability Calculation Function</summary>  
/// <param name="W">Minimum backoff window size</param> 
/// <param name="m">Maximum stage number</param> 
/// <param name="n">Maximum stations number</param> 
/// <returns> </returns>  
double rootOfCollisionProbability(int W, int m, int n) {
	double result = 0.4;
	double a = 0; 
	double b = 1;
	while (std::abs(collisionProbability(result, W, m, n)) > EPSILON)	{
		if (collisionProbability(result, W, m, n) * collisionProbability(a, W, m, n) < 0.0) {
			b = result;
		}
		else {
			a = result;
		}
		result = 0.5 * (a + b);
	}
	return result;
}