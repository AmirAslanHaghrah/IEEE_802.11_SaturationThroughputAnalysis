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
#include <functional>
#include <vector>
#include <random>
#include <algorithm>

#define EPSILON 0.00000001f

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

	int Packet = MAC_header + PHY_header + PayLoad;			// bits
	int ACK = 112 + PHY_header;								// bits

	double Ts = Packet + SIFS + prop_delay + ACK + DIFS + prop_delay;
	double Tc = Packet + DIFS + prop_delay;

	// Set the window size and stage number by typing inputs
	int W = 32;
	int m = 3;
	std::cout << "Minimum backoff window size: (in slot time)\n";
	std::cin >> W;
	std::cout << "Maximum stage number: \n";
	std::cin >> m;

	// Computation
	double throughput[50] = {};

	for (int n = 5; n <= 50; n++) {
		// P is the probability that transmitted packet collide
		double P = rootOfCollisionProbability(W, m, n);
		// tau is probability that a station transmits in a generic slot time
		double tau = 2.0f * (1.0f - 2.0f * P) / ((1.0f - 2.0f * P) * (W + 1.0f) + P * W *(1.0f - std::pow((2.0f * P), m)));
		// Ptr is that in a slot time there is at least one transmission
		double Ptr = 1.0f - std::pow((1.0f - tau), n);
		// Ps is the probability that a transmission is successful
		double Ps = n * tau * (std::pow((1.0f - tau), (n - 1))) / Ptr;	
		throughput[n - 5] = Ps * Ptr * PayLoad / ((1 - Ptr) * slot_time + Ptr * Ps * Ts + Ptr * (1.0f - Ps) * Tc);
	}


	
	long sim_time = 0;
	double Simulation_throughput[50] = {};
	//int j = 0;

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	// Simulation
	std::cout << "Simulating...\n";

	for (int num_station = 5; num_station <= 50; num_station++) {
		long suc_pkt = 0;
		long collision_pkt = 0;

		std::vector<long> station_stage;
		station_stage.resize(num_station, 0);

		std::vector<long> next_tran_time;
		next_tran_time.resize(num_station, 0);

		// initial backoff time
		for (int i = 0; i < num_station; i++) {
			station_stage[i] = 0;
			next_tran_time[i] = DIFS + std::floor(W * distribution(generator)) * slot_time;
		}


		while (suc_pkt < 100000) {
			long tran_time = *std::min_element (next_tran_time.begin(), next_tran_time.end());
			long no_tran = 0;
			for (int i = 0; i < next_tran_time.size(); i++) {
				if (tran_time == next_tran_time[i]) {
					no_tran++;
				}
			}

			// sucessful reansition
			if (no_tran == 1) {
				suc_pkt++;
				for (int i = 0; i < num_station; i++) {
					// uniform backoff at stage 0
					if (next_tran_time[i] == tran_time) {
						if (suc_pkt == 100000) {
							sim_time = next_tran_time[i] + Packet + SIFS + ACK + DIFS;
						}
						station_stage[i] = 0;
						next_tran_time[i] = next_tran_time[i] + Packet + SIFS + ACK + DIFS + floor(W * distribution(generator)) * slot_time;
					}
					// stop counting while the channel is busy
					else {
						next_tran_time[i] = next_tran_time[i] + Packet + SIFS + ACK + DIFS;
					}
				}
			}
			else {
				collision_pkt++;
				for (int i = 0; i < num_station; i++) {
					// uniform backoff at next stage
					if (next_tran_time[i] == tran_time) {
						if (station_stage[i] < m) {
							station_stage[i]++;
						}
						next_tran_time[i] = next_tran_time[i] + Packet + DIFS + floor(std::pow(2.0f, station_stage[i]) * W * distribution(generator)) * slot_time;
					}
					else {
						next_tran_time[i] = next_tran_time[i] + Packet + DIFS;
					}
				}
			}
		}
		Simulation_throughput[num_station - 5] = (float)suc_pkt * (PayLoad) / sim_time;
	}


	//Plot the results
	std::vector<float> x;
	std::vector<float> y1, y2;

	for (int i = 0; i < 50; i++) {
		x.push_back(i);
		y1.push_back(throughput[i]);
		y2.push_back(Simulation_throughput[i]);
	}

	FILE * gp = _popen("gnuplot", "w");
	fprintf(gp, "set terminal wxt size 600,400 \n");
	fprintf(gp, "set grid \n");
	fprintf(gp, "set title '%s' \n", "Throughput");
	fprintf(gp, "set style line 1 lt 3 pt 7 ps 0.1 lc rgb 'green' lw 1 \n");
	fprintf(gp, "set style line 2 lt 3 pt 7 ps 0.1 lc rgb 'red' lw 1 \n");
	fprintf(gp, "plot '-' w p ls 1, '-' w p ls 2 \n");

	for (int k = 0; k < x.size(); k++) {
		fprintf(gp, "%f %f \n", x[k], y1[k]);
	}
	fprintf(gp, "e\n");

	for (int k = 0; k < x.size(); k++) {
		fprintf(gp, "%f %f \n", x[k], y2[k]);
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