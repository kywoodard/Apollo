#include <iostream>
#include <fstream>
#include <boost/numeric/odeint.hpp>
#include <math.h>
#include <algorithm>

#define PI 3.14159265

typedef std::vector< double > state_type;
struct Constants
{
	double gamma = 1000.0;
	double grav = 9.81;
	double L = 10.0;
	double r = 2.0;
	double m_nofuel = 25.0;
	double max_fuel = 6.0*this->m_nofuel;
	double J = 1.0/12.0*(this->m_nofuel+this->max_fuel)*pow((2.0*this->L),2.0);
	double JT = (83.0/320.0 + 1.0)*0.05*this->m_nofuel*pow(this->r,2.0);
	double max_y = 20.0;
	double max_z = this-> L;
	double max_theta = PI/6.0;
	double max_speed = 5.0;
	double max_dtheta = 1.0;
	double max_tau = 10.0;
	double max_fT = 25.0;

};

struct Controls
{
	double thing = 0;
};

class rocket_ode
{
	Constants consts;
	Controls ctrl;
public:
	rocket_ode(Constants con_in, Controls control) : consts(con_in), 
				ctrl(control){}
	void operator()(const state_type &x, state_type &dxdt, const double t)
	{
		double y,z,th,psi,dy,dz,dth,dpsi,m;
		y = x[0];
		z = x[1];
		th = x[2];
		psi = x[3];

		dy = x[4];
		dz = x[5];
		dth = x[6];
		dpsi = x[7];
		
		m = x[8];
		
		std::vector< double> f_vec(9);
		f_vec[0] = dy;
		f_vec[1] = dz;
		f_vec[2] = dth;
		f_vec[3] = dpsi;
		f_vec[4] = 0;
		f_vec[5] = -consts.grav;
		f_vec[6] = 0;
		f_vec[7] = 0;
		f_vec[8] = 0;
		
		std::vector< double> g1_vec(9);
		g1_vec[0] = 0;
		g1_vec[1] = 0;
		g1_vec[2] = 0;
		g1_vec[3] = 0;
		g1_vec[4] = -consts.gamma*sin(psi+th)/m;
		g1_vec[5] = consts.gamma*cos(psi+th)/m;
		g1_vec[6] = -consts.L*consts.gamma*sin(psi)/consts.J;
		g1_vec[7] = 0;
		g1_vec[8] = -1;

		std::vector< double> g2_vec(9);
		g2_vec[0] = 0;
		g2_vec[1] = 0;
		g2_vec[2] = 0;
		g2_vec[3] = 0;
		g2_vec[4] = 0;
		g2_vec[5] = 0;
		g2_vec[6] = 0;
		g2_vec[7] = 1/consts.JT;
		g2_vec[8] = 0;

		std::vector <double> u(2);
		controller(t,x,consts, ctrl, u);

		for(int i=0;i<=8;i++)
		{
			dxdt[i] = f_vec[i]+ g1_vec[i]*u[0] + g2_vec[i]*u[1];
			
		}
	}
	static void controller(const double &t, const state_type &x,
		const Constants &consts, const Controls &ctrl, std::vector<double> &u)
	{
		// Put the controller equations here
		double y = x[0];
		double z = x[1];
		double theta = x[2];
		double psi = x[3];
		double dy = x[4];
		double dz = x[5];
		double dtheta = x[6];
		double dpsi = x[7];
		double m = x[8];

		double k1 = 1.0;
		double k2 = 1.0;
		double k3 = 1.0;
		double k4 = 1.0;

		double v1 = 2*theta;
		double v2 = 2*dtheta;
		double v3 = 0.0;
		double v4 = 0.0;

		// double ref_z = 0.;
		// double ref_dz = 0.;
		// double error_z = x[1] - ref_z;
		// double error_dz = x[5] - ref_dz;


		// double k_p = 0.01;
		// double k_d = 0.2;

		// u[0] = -(k_p*error_z) - (k_d*error_dz);
		// u[1] = 0.0;

		// TRYING TO LIMIT SINE OUTSIDE OF +/- 0.0001 HERE
		double sin_limitted = std::min(std::max(sin(psi),0.001),-0.0001);

		// u[0] = -consts.J/(consts.L*consts.gamma)*1*(-k3*(theta-v3) - k4*(dtheta-v4));
		// u[1] = -k1*(theta+psi-v1) - k2*(dtheta+dpsi-v2);

		u[0] = 0.0;
		u[1] = 0.0;

		if(static_cast<double>(x[8]) <= consts.m_nofuel)
		{
			u[0] = 0;
		}
		u[0] = std::min(std::max(u[0],0.0),consts.max_fT);
		u[1] = std::min(std::max(u[1],-consts.max_tau), consts.max_tau);
	}

};

struct push_back_state_and_time
{
	std::vector< state_type >& m_states;
	std::vector< double >& m_times;

	push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
	: m_states( states ) , m_times( times ) { }

	void operator()( const state_type &x , double t )
	{
		m_states.push_back( x );
		m_times.push_back( t );
	}
};

int main(int argc, char** argv)
{
	state_type x(9);
	if(argc > 1)
	{
		for(int i=0;i<=8;i++)
		{
			x[i] = atof(argv[i+1]);
		}
	}
	else
	{
		x[0] = 0.0;		//y
		x[1] = 1000;	//z
		x[2] = 0.0;		//theta
		x[3] = 0.0;		//psi
		x[4] = 0.0;		//dy
		x[5] = 0.0;		//dz
		x[6] = 0.0;		//dtheta
		x[7] = 0.0;		//dpsi
		x[8] = 175.0;	//m
	}

	std::vector<state_type> x_vec;
	std::vector<double> times;

	//Integration
	Constants consts;
	Controls ctrl;
	rocket_ode rocket(consts, ctrl);
	typedef boost::numeric::odeint::runge_kutta_cash_karp54< state_type > 
	rkck54;
	typedef boost::numeric::odeint::controlled_runge_kutta< rkck54 > 
	ctrl_rkck54;
	size_t steps = boost::numeric::odeint::integrate_const(ctrl_rkck54(),
		rocket,x, 0.0, 60.0, 0.01, push_back_state_and_time( x_vec , times ));

	std::ofstream outputFile;
	outputFile.open("rocket_ODE_output.txt");
	std::vector <double> u(2);
	for( size_t i=0; i<=steps; i++ )
	{	
		outputFile << times[i];
		for( int j=0; j<x.size(); j++)
		{
			outputFile << '\t' << x_vec[i][j];
		}
		rocket_ode::controller(times[i],x_vec[i],consts, ctrl, u);
		outputFile << '\t' << u[0];
		outputFile << '\t' << u[1];
		if( i < steps)
		{
			outputFile << '\n';
		}
	}



	// std::vector<state_type> x_vec;
 //    std::vector<double> times;
	// steps = boost::numeric::odeint::integrate( rocket ,
 //            x , 0.0 , 10.0 , 0.1 ,
 //            push_back_state_and_time( x_vec , times ) );

	// for( size_t i=0; i<=steps; i++ )
 //    {
 //        std::cout << steps << '\t' << times[i] << '\t' << x_vec[i][0] << '\t' << x_vec[i][1] << '\n';
 //    }

}
