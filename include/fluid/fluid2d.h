/*
Modified from Jos Stam "Real-Time Fluid Dynamics for Games". 
*/

#ifndef FLUID2D_DYNAMICS
#define FLUID2D_DYNAMICS

#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>

/* macros */

namespace fluid
{

#define IX(i,j) ((i)+(N+2)*(j))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) {
#define END_FOR }}

/* global variables */

int N;
float dt, diff, visc;
float force, source;
int dvel;

float * u, * v, * u_prev, * v_prev;
float * dens, * dens_prev;

int win_id;
int win_x, win_y;
int mouse_down[3];
int omx, omy, mx, my;


/*
----------------------------------------------------------------------
main dynamic functions
----------------------------------------------------------------------
*/

/////////////////////////////////////////////////


void add_source ( int N, float * x, float * s, float dt )
{
  int i, size=(N+2)*(N+2); 
  for ( i=0 ; i<size ; i++ ) x[i] += dt*s[i];
}

void set_bnd ( int N, int b, float * x )
{
  int i;

  for ( i=1 ; i<=N ; i++ ) {
    x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];
    x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];
    x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];
    x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];
  }
  x[IX(0  ,0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0  ,1)]);
  x[IX(0  ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0  ,N)]);
  x[IX(N+1,0  )] = 0.5f*(x[IX(N,0  )]+x[IX(N+1,1)]);
  x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);
}

void lin_solve ( int N, int b, float * x, float * x0, float a, float c )
{
  int i, j, k;

  for ( k=0 ; k<20 ; k++ ) {
    FOR_EACH_CELL
      x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
    END_FOR 
    set_bnd ( N, b, x );
  }
}

void diffuse ( int N, int b, float * x, float * x0, float diff, float dt )
{
  float a=dt*diff*N*N; 
  lin_solve ( N, b, x, x0, a, 1+4*a );
}

void advect ( int N, int b, float * d, float * d0, float * u, float * v, float dt )
{
  int i, j, i0, j0, i1, j1;
  float x, y, s0, t0, s1, t1, dt0;

  dt0 = dt*N;
  FOR_EACH_CELL
    x = i-dt0*u[IX(i,j)]; y = j-dt0*v[IX(i,j)];
    if (x<0.5f) x=0.5f; if (x>N+0.5f) x=N+0.5f; i0=(int)x; i1=i0+1;
    if (y<0.5f) y=0.5f; if (y>N+0.5f) y=N+0.5f; j0=(int)y; j1=j0+1;
    s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
    d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])+
				 s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
  END_FOR
  set_bnd ( N, b, d );
}

void project ( int N, float * u, float * v, float * p, float * div )
{
  int i, j;

  FOR_EACH_CELL
    div[IX(i,j)] = -0.5f*(u[IX(i+1,j)]-u[IX(i-1,j)]+v[IX(i,j+1)]-v[IX(i,j-1)])/N;
    p[IX(i,j)] = 0;
  END_FOR
  set_bnd ( N, 0, div ); set_bnd ( N, 0, p );

  lin_solve ( N, 0, p, div, 1, 4 );

  FOR_EACH_CELL
    u[IX(i,j)] -= 0.5f*N*(p[IX(i+1,j)]-p[IX(i-1,j)]);
    v[IX(i,j)] -= 0.5f*N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
  END_FOR
  set_bnd ( N, 1, u ); set_bnd ( N, 2, v );
}

void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt )
{
  add_source ( N, x, x0, dt );
  SWAP ( x0, x ); diffuse ( N, 0, x, x0, diff, dt );
  SWAP ( x0, x ); advect ( N, 0, x, x0, u, v, dt );
}

void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt )
{
  add_source ( N, u, u0, dt ); add_source ( N, v, v0, dt );
  SWAP ( u0, u ); diffuse ( N, 1, u, u0, visc, dt );
  SWAP ( v0, v ); diffuse ( N, 2, v, v0, visc, dt );
  project ( N, u, v, u0, v0 );
  SWAP ( u0, u ); SWAP ( v0, v );
  advect ( N, 1, u, u0, u0, v0, dt ); advect ( N, 2, v, v0, u0, v0, dt );
  project ( N, u, v, u0, v0 );
}



/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/


void free_data ( void )
{
  if ( u ) free ( u );
  if ( v ) free ( v );
  if ( u_prev ) free ( u_prev );
  if ( v_prev ) free ( v_prev );
  if ( dens ) free ( dens );
  if ( dens_prev ) free ( dens_prev );
}

void clear_data ( void )
{
  int i, size=(N+2)*(N+2);

  for ( i=0 ; i<size ; i++ ) {
    u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
  }
}

int allocate_data ( void )
{
  int size = (N+2)*(N+2);

  u		= (float *) malloc ( size*sizeof(float) );
  v		= (float *) malloc ( size*sizeof(float) );
  u_prev	= (float *) malloc ( size*sizeof(float) );
  v_prev	= (float *) malloc ( size*sizeof(float) );
  dens		= (float *) malloc ( size*sizeof(float) );	
  dens_prev	= (float *) malloc ( size*sizeof(float) );

  if ( !u || !v || !u_prev || !v_prev || !dens || !dens_prev ) {
    fprintf ( stderr, "cannot allocate data\n" );
    return ( 0 );
  }

  return ( 1 );
}


/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

void pre_display ( void )
{
  glViewport ( 0, 0, win_x, win_y );
  glMatrixMode ( GL_PROJECTION );
  glLoadIdentity ();
  gluOrtho2D ( 0.0, 1.0, 0.0, 1.0 );
  glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
  glClear ( GL_COLOR_BUFFER_BIT );
}

void post_display ( void )
{
  glutSwapBuffers ();
}

void draw_velocity ( void )
{
  int i, j;
  float x, y, h;

  h = 54.0f/N;

  glColor3f ( 1.0f, 1.0f, 1.0f );
  glLineWidth ( 1.0f );

  glBegin ( GL_LINES );

    for ( i=1 ; i<=N ; i++ ) {
      x = (i-0.5f)*h -27;
      for ( j=1 ; j<=N ; j++ ) {
	y = (j-0.5f)*h -27;

	//glVertex2f ( x, y );
	//glVertex2f ( x+u[IX(i,j)], y+v[IX(i,j)] );
	glVertex3f ( x, y, 0.1);
	glVertex3f ( x+10*u[IX(i,j)], y+10*v[IX(i,j)],0.1 );
	//printf("%f %f; %f %f\n", x, y, x+10*u[IX(i,j)], y+10*v[IX(i,j)]);
      }
    }

  glEnd ();
}

void draw_density ( void )
{
  int i, j;
  float x, y, h, d00, d01, d10, d11;

  h = 1.0f/N;

  glBegin ( GL_QUADS );

    for ( i=0 ; i<=N ; i++ ) {
      x = (i-0.5f)*h;
      for ( j=0 ; j<=N ; j++ ) {
	y = (j-0.5f)*h;

	d00 = dens[IX(i,j)];
	d01 = dens[IX(i,j+1)];
	d10 = dens[IX(i+1,j)];
	d11 = dens[IX(i+1,j+1)];

	glColor3f ( d00, d00, d00 ); glVertex2f ( x, y );
	glColor3f ( d10, d10, d10 ); glVertex2f ( x+h, y );
	glColor3f ( d11, d11, d11 ); glVertex2f ( x+h, y+h );
	glColor3f ( d01, d01, d01 ); glVertex2f ( x, y+h );
      }
    }

  glEnd ();
}

/*
----------------------------------------------------------------------
relates mouse movements to forces sources
----------------------------------------------------------------------
*/

void get_from_UI ( float * d, float * u, float * v )
{
  int i, j, size = (N+2)*(N+2);

  for ( i=0 ; i<size ; i++ ) {
    u[i] = v[i] = d[i] = 0.0f;
  }

  if ( !mouse_down[0] && !mouse_down[2] ) return;

  i = (int)((       mx /(float)win_x)*N+1);
  j = (int)(((win_y-my)/(float)win_y)*N+1);

  if ( i<1 || i>N || j<1 || j>N ) return;

  if ( mouse_down[0] ) {
    //u[IX(i,j)] = force * (mx-omx);
    //u[IX(i,j)] = -1*force;
    //v[IX(i,j)] = force * (omy-my);
    //v[IX(i,j)] = 0; //-1*force;
    u[IX(4,5)] = 10*force;
    u[IX(14,14)] = -10*force;
  }

  if ( mouse_down[2] ) {
    d[IX(i,j)] = source;
  }

  omx = mx;
  omy = my;

  return;
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/
void key_func ( unsigned char key, int x, int y )
{
  switch ( key )
  {
    case 'c':
    case 'C':
      clear_data ();
      break;
    case 'v':
    case 'V':
      dvel = !dvel;
      break;
  }
}

void mouse_func ( int button, int state, int x, int y )
{
  omx = mx = x;
  omx = my = y;

  mouse_down[button] = state == GLUT_DOWN;
}

void motion_func ( int x, int y )
{
  mx = x;
  my = y;
}

/*
void reshape_func ( int width, int height )
{
  glutSetWindow ( win_id );
  glutReshapeWindow ( width, height );

  win_x = width;
  win_y = height;
}
*/

void idle_func( void )
{
  get_from_UI ( dens_prev, u_prev, v_prev );
  vel_step ( N, u, v, u_prev, v_prev, visc, dt );
  dens_step ( N, dens, dens_prev, u, v, diff, dt );

  //glutSetWindow ( win_id );
  //glutPostRedisplay ();
}

void display_func ( void )
{
  pre_display ();

    if ( dvel ) draw_velocity ();
    else	draw_density ();

  post_display ();
}


void init (void){

  N = 18;
  dt = 0.1f;
  diff = 0.0f;
  visc = 0.0f;
  force = 5.0f;
  source = 100.0f;
  if ( !allocate_data () ) exit ( 1 );
  clear_data ();

  win_x = 512;
  win_y = 512;

}



/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/ 
/*
void open_glut_window ( void )
{
  glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

  glutInitWindowPosition ( 0, 0 );
  glutInitWindowSize ( win_x, win_y );
  win_id = glutCreateWindow ( "Alias | wavefront" );

  glClearColor ( 1.0f, 0.0f, 0.0f, 1.0f );
  glClear ( GL_COLOR_BUFFER_BIT );
  glutSwapBuffers ();
  glClear ( GL_COLOR_BUFFER_BIT );
  glutSwapBuffers ();

  pre_display ();

  glutKeyboardFunc ( key_func );
  glutMouseFunc ( mouse_func );
  glutMotionFunc ( motion_func );
  glutReshapeFunc ( reshape_func );
  glutIdleFunc ( idle_func );
  glutDisplayFunc ( display_func );
}
*/


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

/*
int main ( int argc, char ** argv )
{
  glutInit ( &argc, argv );

  if ( argc != 1 && argc != 6 ) {
    fprintf ( stderr, "usage : %s N dt diff visc force source\n", argv[0] );
    fprintf ( stderr, "where:\n" );\
    fprintf ( stderr, "\t N      : grid resolution\n" );
    fprintf ( stderr, "\t dt     : time step\n" );
    fprintf ( stderr, "\t diff   : diffusion rate of the density\n" );
    fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
    fprintf ( stderr, "\t force  : scales the mouse movement that generate a force\n" );
    fprintf ( stderr, "\t source : amount of density that will be deposited\n" );
    exit ( 1 );
  }

  if ( argc == 1 ) {
    N = 20;
    dt = 0.1f;
    diff = 0.0f;
    visc = 0.0f;
    force = 5.0f;
    source = 100.0f;
    fprintf ( stderr, "Using defaults : N=%d dt=%g diff=%g visc=%g force = %g source=%g\n",
	    N, dt, diff, visc, force, source );
  } else {
    N = atoi(argv[1]);
    dt = atof(argv[2]);
    diff = atof(argv[3]);
    visc = atof(argv[4]);
    force = atof(argv[5]);
    source = atof(argv[6]);
  }

  printf ( "\n\nHow to use this demo:\n\n" );
  printf ( "\t Add densities with the right mouse button\n" );
  printf ( "\t Add velocities with the left mouse button and dragging the mouse\n" );
  printf ( "\t Toggle density/velocity display with the 'v' key\n" );
  printf ( "\t Clear the simulation by pressing the 'c' key\n" );
  printf ( "\t Quit by pressing the 'q' key\n" );

  dvel = 0;

  if ( !allocate_data () ) exit ( 1 );
  clear_data ();

  win_x = 512;
  win_y = 512;
  open_glut_window ();

  glutMainLoop ();

  exit ( 0 );
}
*/

}//end namespace

#endif
