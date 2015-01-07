// krunge.cpp //////////////////////////////////////////////////////////////////
// A test bed for writing a RK4 state propagator ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//#include <std_files>
//#include "Headers.h"
//#include "Headers.hpp"
//#include "Source.cpp"

#include <iostream>

typedef long double fpoint;
// we'll just abstract this here because we can

struct State
{	fpoint x;
	// position in x axis
	fpoint v;
	// velocity along x
};

struct Derivative
{	fpoint dx;
	// where dx/dt is our velocity along x
	fpoint dv;
	// where dv/dt is our acceleration along x
};
// the struct/derivative pair is a bit odd to be honest, in practice, I think
// my Ignition implementation of this will just pass the state vectors along
// with a acceleration vector in the scope of the function

fpoint Acceleration(State current_state, fpoint current_time);
// this is a wee bit hairy in the case of Ignition, the question is whether
// we do this for every component on the vessel, or just gravity. The simple
// answer is just gravity, since its the only one that really has issues with
// ignoring curvature, but some of the other areas do show a fair amount of
// error depending on the frame rate (engines gave different delta vees
// depending on the time accel

// but the problem is that we would need to update the vessels state from the
// status of the ships parts multiple times... hmm
// I just dont see any good solutions to this, at least not for rocket engines
// & their fuel tanks. I think theyll have to be done with euler integrators as
// constant over the step instead. It just wont work with an RK4 integrator
// not as far as I can see.

// the aerodynamics, on the other hand might work just fine here, as we could
// just update rotation with a simple euler integrator, and each part can put
// in its contribution to acceleration in each call to evaluate based on
// whatever its drag should be based on rotation, planet altitude, etc.
// although this opens up its own can of worms WRT how the part is going to
// retrieve planet altitude


Derivative evaluate(const State &initial_state, fpoint t, fpoint dt, const Derivative &derivative)
{	State state;
	state.x = initial_state.x + derivative.dx*dt;
	// I guess the last part is because its actually a*dt^2 or 
	// (dx/dt)*dt^2 = dx*dt
	state.v = initial_state.v + derivative.dv*dt;
	// for some reason we step things forward 1 frame dt seconds using a Euler
	// integrator
	
	Derivative output;
	output.dx = state.v;
	// we set the output derivatives dx equal to the velocity of the state that
	// we stepped forward earlier using the euler step
	output.dv = Acceleration(state, (t+dt));
	// and lastly we set dv equal to whatever the current acceleration should
	// really be 
	return output;
}

fpoint Acceleration(State current_state, fpoint current_time)
{	// I dunno do whatever the hell we want here
	// we'll do what the article suggested, use an example with a damped
	// harmonic oscillator spring
	const fpoint k = 10;
	// the spring constant I guess?
	const fpoint b = 1;
	// I think this is the coefficient for the retarding force?
	// looks an awful lot like air drag when the form is (b/m)*v
	// whatever
	return ((-k)*(current_state.x - (b*current_state.v)));
}

void Integrate(State &state, fpoint t, fpoint dt)
{
	Derivative a, b, c, d;
	// weird, I didnt think this would be a Derivative type here
	
	a = evaluate(state, t, 0.0f, Derivative());
	// say again Houston??? WTF is this???
	// I didnt even think you could declare a struct like that
	
	// I guess.... I think the idea here is that we  get our initial derivative
	// state of a from the initial state that was passed to the function, and
	// dont step forward at all. We're just getting an initial state here
	b = evaluate(state, t, dt*(0.5f), a);
	// the previous operations insanity notwithstanding, we do the same thing
	// again using whatever we got back from a
	
	// so we are working forward along the 'curvature of the problem', moving
	// forward and sampling acceleration and velocity based on what the initial
	// state obtained for a gave us for a half frame in length
	c = evaluate(state, t, dt*(0.5f), b);
	// and again with the results from b to get a.
	// evaluating with the half frame velocities and accels here?
	// I guess we equally weight the start and halfway vels and accels here???
	d = evaluate(state, t, dt, c);
	// and... one last time with c
	
	//and at last, we get the vels and accels at the end of the frame, now this
	// makes sense
	
	fpoint velocity = ((1.0/6.0)*(a.dx + 2.0*(b.dx + c.dx) + d.dx));
	fpoint accel = ((1.0/6.0)*(a.dv + 2.0*(b.dv + c.dv) + d.dv));
	// we do some odd thing here to get a frame-wide accel and velocity?
	// I think? I do recognize these as the RK4 scheme coefficients
	
	// looks like we weight them according to some set of coefficients that the
	// given order of Runge Kutta specifies, and the velocity and accel that
	// result are applied to the frame with a regular euler integration step,
	// minus the position term for constant acceleration
	// (already handled by the weighted velocity changes to the state I guess)   
	
	state.x += (velocity*dt);
	state.v += (accel*dt);
	// and we lastly step ahead the basic state of the system by the values
	// we calculated as velocity and accel for the frame? Yeah, that sounds
	// right
	
}

int main()
{
}
