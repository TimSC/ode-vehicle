# pyODE example 3: Collision detection

# Originally by Matthias Baas.
# Updated by Pierre Gay to work without pygame or cgkit.

import sys, os, random, time
from math import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import ode

# geometric utility functions
def scalp (vec, scal):
	vec[0] *= scal
	vec[1] *= scal
	vec[2] *= scal

def length (vec):
	return sqrt (vec[0]**2 + vec[1]**2 + vec[2]**2)

# prepare_GL
def prepare_GL():
	"""Prepare drawing.
	"""

	# Viewport
	glViewport(0,0,640,480)

	# Initialize
	glClearColor(0.8,0.8,0.9,0)
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST)
	glDisable(GL_LIGHTING)
	glEnable(GL_LIGHTING)
	glEnable(GL_NORMALIZE)
	glShadeModel(GL_FLAT)

	# Projection
	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()
	gluPerspective (45,1.3333,0.2,20)

	# Initialize ModelView matrix
	glMatrixMode(GL_MODELVIEW)
	glLoadIdentity()

	# Light source
	glLightfv(GL_LIGHT0,GL_POSITION,[0,0,1,0])
	glLightfv(GL_LIGHT0,GL_DIFFUSE,[1,1,1,1])
	glLightfv(GL_LIGHT0,GL_SPECULAR,[1,1,1,1])
	glEnable(GL_LIGHT0)

	# View transformation
	gluLookAt (2.4, 3.6, 4.8, 0.5, 0.5, 0, 0, 1, 0)

class Box:
	def __init__(self, world, space, density, lx, ly, lz):
		"""Create a box body and its corresponding geom."""

		# Create body
		self.body = ode.Body(world)
		M = ode.Mass()
		M.setBox(density, lx, ly, lz)
		self.body.setMass(M)

		# Set parameters for drawing the body
		self.boxsize = (lx, ly, lz)

		# Create a box geom for collision detection
		self.geom = ode.GeomBox(space, lengths=self.boxsize)
		self.geom.setBody(self.body)
		self.geom.ty = "box"

	def Draw(self):
		x,y,z = self.body.getPosition()
		R = self.body.getRotation()
		rot = [R[0], R[3], R[6], 0.,
			   R[1], R[4], R[7], 0.,
			   R[2], R[5], R[8], 0.,
			   x, y, z, 1.0]
		glPushMatrix()
		glMultMatrixd(rot)

		sx,sy,sz = self.boxsize
		glScalef(sx, sy, sz)
		glutSolidCube(1)

		glPopMatrix()

	def setPosition(self, pos):
		self.body.setPosition(pos)

	def setRotation(self, rot):
		self.body.setRotation(rot)

	def getPosition(self):
		return self.body.getPosition()

	def getRotation(self):
		return self.body.getRotation()

	def addForce(self, f):
		self.body.addForce(f)

	def Test(self):
		pass

class Ball:
	def __init__(self, world, space, density, rad):
		"""Create a box body and its corresponding geom."""

		# Create body
		self.body = ode.Body(world)
		M = ode.Mass()
		M.setSphere(density, rad)
		self.body.setMass(M)

		# Set parameters for drawing the body
		self.radius = rad

		# Create a box geom for collision detection
		self.geom = ode.GeomSphere(space, self.radius)
		self.geom.setBody(self.body)
		self.geom.ty = "ball"

	def Draw(self):
		x,y,z = self.body.getPosition()
		R = self.body.getRotation()
		rot = [R[0], R[3], R[6], 0.,
			   R[1], R[4], R[7], 0.,
			   R[2], R[5], R[8], 0.,
			   x, y, z, 1.0]
		glPushMatrix()
		glMultMatrixd(rot)

		glScalef(self.radius, self.radius, self.radius)
		glutSolidSphere(1, 10, 10)

		glPopMatrix()

	def setPosition(self, pos):
		self.body.setPosition(pos)

	def setRotation(self, rot):
		self.body.setRotation(rot)

	def getPosition(self):
		return self.body.getPosition()

	def getRotation(self):
		return self.body.getRotation()

	def addForce(self, f):
		self.body.addForce(f)

	def Test(self):
		pass

class Cylinder:
	def __init__(self, world, space, density, rad):

		self.l = 1.

		# Create body
		self.body = ode.Body(world)
		M = ode.Mass()
		M.setCappedCylinder(density, 3, rad, self.l)
		self.body.setMass(M)

		# Set parameters for drawing the body
		self.radius = rad

		# Create a box geom for collision detection
		self.geom = ode.GeomCapsule(space, self.radius, self.l)
		self.geom.setBody(self.body)
		self.geom.ty = "cylinder"

	def Draw(self):
		x,y,z = self.body.getPosition()
		R = self.body.getRotation()
		rot = [R[0], R[3], R[6], 0.,
			   R[1], R[4], R[7], 0.,
			   R[2], R[5], R[8], 0.,
			   x, y, z, 1.0]
		glPushMatrix()
		glMultMatrixd(rot)

		gluCylinder(gluNewQuadric(), self.radius, self.radius, self.l, 10, 3)


		glPopMatrix()

	def setPosition(self, pos):
		self.body.setPosition(pos)

	def setRotation(self, rot):
		self.body.setRotation(rot)

	def getPosition(self):
		return self.body.getPosition()

	def getRotation(self):
		return self.body.getRotation()

	def addForce(self, f):
		self.body.addForce(f)

	def Test(self):
		pass

class Composite:
	def __init__(self, world, space, density, rad):

		# Set parameters for drawing the body
		self.radius = rad

		# Create body
		self.body1 = ode.Body(world)
		M = ode.Mass()
		M.setCappedCylinder(density, 3, self.radius, 1.)
		self.body1.setMass(M)

		self.body2 = ode.Body(world)
		M = ode.Mass()
		M.setCappedCylinder(density, 3, self.radius, 1.)
		self.body2.setMass(M)

		self.body3 = ode.Body(world)
		M = ode.Mass()
		M.setCappedCylinder(density * 10., 3, self.radius, 1.)
		self.body3.setMass(M)

		self.setPosition((0.,0.,0.))

		# Connect body2 with body1
		#self.j2 = ode.SliderJoint(world)
		#self.j2.attach(self.body1, self.body2)
		#self.j2.setAnchor( (self.radius*2.1,0.,0.) )
		self.joint = ode.AMotor(world)
		self.joint.attach(self.body3, self.body2)
		self.joint.setNumAxes(1)
		#self.joint.setMode(ode. AMotorEuler)
		self.joint.setAxis(0, ode.AMotorEuler, (0., 0., 1.))
		
		# Create a box geom for collision detection
		self.geom1 = ode.GeomCapsule(space, self.radius, 1.)
		self.geom1.setBody(self.body1)
		self.geom2 = ode.GeomCapsule(space, self.radius, 1.)
		self.geom2.setBody(self.body2)
		self.geom3 = ode.GeomCapsule(space, self.radius, 1.)
		self.geom3.setBody(self.body3)
		self.geom1.ty = "composite"

	def DrawBall(self, obj):
		x,y,z = obj.getPosition()
		R = obj.getRotation()
		rot = [R[0], R[3], R[6], 0.,
			   R[1], R[4], R[7], 0.,
			   R[2], R[5], R[8], 0.,
			   x, y, z, 1.0]
		glPushMatrix()
		glMultMatrixd(rot)

		glScalef(self.radius, self.radius, self.radius)
		gluCylinder(gluNewQuadric(), self.radius, self.radius, 1., 10, 2)

		glPopMatrix()

	def Draw(self):
		self.DrawBall(self.body1)
		self.DrawBall(self.body2)
		self.DrawBall(self.body3)

	def setPosition(self, pos):
		#self.body1.setPosition(pos)
		#self.body2.setPosition((pos[0]+self.radius*2.1,pos[1],pos[2]))
		#self.body3.setPosition((pos[0]+self.radius*1.05,pos[1]+self.radius*1.05,pos[2]))
		self.body1.setPosition(pos)
		self.body2.setPosition((pos[0],pos[1]+self.radius*2.05,pos[2]))
		self.body3.setPosition((pos[0],pos[1]+self.radius*4.1,pos[2]))

	def setRotation(self, rot):
		self.body1.setRotation(rot)
		self.body2.setRotation(rot)
		self.body3.setRotation(rot)

	def getPosition(self):
		return self.body1.getPosition()

	def getRotation(self):
		return self.body1.getRotation()

	def addForce(self, f):
		self.body1.addForce(f)
		self.body2.addForce(f)

	def Test(self):
		#self.body1.addTorque((100.0,0.,0.))
		#self.body2.addTorque((100.0,0.,0.))
		self.joint.addTorques(200.,0.,0.)
		pass

# drop_object
def drop_object():
	"""Drop an object into the scene."""

	global bodies, geom, counter, objcount, objs

	#if random.randint(0,2) % 2 == 0:
	#	body, geom = create_box(world, space, 1000, 1.0,0.2,0.2)
	#else:
	#	body, geom = create_ball(world, space, 1000, 0.4)
	#if random.randint(0,2) % 2 == 0:
	#	obj = Box(world, space, 1000, 1.0,0.2,0.2)
	#else:
	#obj = Box(world, space, 1000, 1.0,0.2,0.2)
	obj = Cylinder(world, space, 1000, 0.1)

	obj.setPosition( (random.gauss(0,0.1),3.0,random.gauss(0,0.1)) )
	#theta = random.uniform(0,2*pi)
	#ct = cos (theta)
	#st = sin (theta)
	#obj.setRotation([ct, 0., -st, 0., 1., 0., st, 0., ct])

	objs.append(obj)
	counter=0
	objcount+=1

# explosion
def explosion():
	"""Simulate an explosion.

	Every object is pushed away from the origin.
	The force is dependent on the objects distance from the origin.
	"""
	global objs

	for obj in objs:
		l= obj.getPosition ()
		d = length (l)
		a = max(0, 40000*(1.0-0.2*d*d))
		l = [l[0] / 4, l[1], l[2] /4]
		scalp (l, a / length (l))
		obj.addForce(l)

# pull
def pull():
	"""Pull the objects back to the origin.

	Every object will be pulled back to the origin.
	Every couple of frames there'll be a thrust upwards so that
	the objects won't stick to the ground all the time.
	"""
	global bodies, counter

	for obj in objs:
		l=list (obj.getPosition ())
		scalp (l, -1000 / length (l))
		obj.addForce(l)
		if counter%60==0:
			obj.addForce((0,10000,0))

# Collision callback
def near_callback(args, geom1, geom2):
	"""Callback function for the collide() method.

	This function checks if the given geoms do collide and
	creates contact joints if they do.
	"""

	# Check if the objects do collide
	contacts = ode.collide(geom1, geom2)

	# Create contact joints
	world,contactgroup = args
	for c in contacts:
		c.setBounce(0.2)
		c.setMu(5000)
		j = ode.ContactJoint(world, contactgroup, c)
		j.attach(geom1.getBody(), geom2.getBody())


######################################################################

# Initialize Glut
glutInit ([])

# Open a window
glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE)

x = 0
y = 0
width = 640
height = 480
glutInitWindowPosition (x, y);
glutInitWindowSize (width, height);
glutCreateWindow ("testode")

# Create a world object
world = ode.World()
world.setGravity( (0,-9.81,0) )
world.setERP(0.8)
world.setCFM(1E-5)

# Create a space object
space = ode.Space()

# Create a plane geom which prevent the objects from falling forever
floor = ode.GeomPlane(space, (0,1,0), 0)

objs = []

# A joint group for the contact joints that are generated whenever
# two bodies collide
contactgroup = ode.JointGroup()

# Some variables used inside the simulation loop
fps = 50
dt = 1.0/fps
running = True
state = 0
counter = 0
objcount = 0
lasttime = time.time()


# keyboard callback
def _keyfunc (c, x, y):
	sys.exit (0)

glutKeyboardFunc (_keyfunc)

# draw callback
def _drawfunc ():
	# Draw the scene
	prepare_GL()
	for obj in objs:
		obj.Draw()

	glutSwapBuffers ()

glutDisplayFunc (_drawfunc)

# idle callback
def _idlefunc ():
	global counter, state, lasttime

	t = dt - (time.time() - lasttime)
	if (t > 0):
		time.sleep(t)

	counter += 1

	if state==0:
		if counter==20:
			drop_object()
		if objcount==100:
			state=1
			counter=0
	# State 1: Explosion and pulling back the objects
	elif state==1:
		#if counter==100:
		#	explosion()
		#if counter>300:
		#	pull()
		if counter==500:
			counter=20

	glutPostRedisplay ()

	# Simulate
	n = 2

	for i in range(n):
		# Detect collisions and create contact joints
		space.collide((world,contactgroup), near_callback)

		#for obj in objs:
		#	obj.Test()

		# Simulation step
		world.step(dt/n)

		# Remove all contact joints
		contactgroup.empty()

	lasttime = time.time()

glutIdleFunc (_idlefunc)

glutMainLoop ()