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

	def UpdateInternalForces(self):
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

	def UpdateInternalForces(self):
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

	def UpdateInternalForces(self):
		pass

class Rod:
	def __init__(self, obj1, obj2):
		self.obj1 = obj1
		self.obj2 = obj2

		self.j = ode.FixedJoint(world)
		self.j.attach(obj1.body, obj2.body)
		self.j.setFixed()

	def setPosition(self, pos):
		pass

	def Draw(self):
		pass

	def UpdateInternalForces(self):
		pass

class Motor:
	def __init__(self, obj, tor, obj2):
		self.torque = tor

		# Create body
		self.body = ode.Body(world)
		M = ode.Mass()
		M.setCappedCylinder(100., 3, 0.1, 1.)
		self.body.setMass(M)
		self.body.setPosition(obj.getPosition())

		self.axle = ode.HingeJoint(world)
		self.axle.attach(obj.body, self.body)
		self.axle.setAxis((0., 0., 1.))

		self.joint = ode.AMotor(world)
		self.joint.attach(obj.body, obj2.body)
		self.joint.setNumAxes(1)
		self.joint.setAxis(0, ode.AMotorEuler, (0., 0., 1.))

	def setPosition(self, pos):
		self.body.setPosition(pos)

	def getPosition(self):
		return self.body.getPosition()

	def Draw(self):
		x,y,z = self.body.getPosition()
		R = self.body.getRotation()
		rot = [R[0], R[3], R[6], 0.,
			   R[1], R[4], R[7], 0.,
			   R[2], R[5], R[8], 0.,
			   x, y, z, 1.0]
		glPushMatrix()
		glMultMatrixd(rot)

		gluCylinder(gluNewQuadric(), 0.1, 0.1, 1.1, 10, 3)
		glPopMatrix()

	def UpdateInternalForces(self):
		self.joint.addTorques(self.torque,0.,0.)

class Composite:
	def __init__(self, world, space, density, rad):

		self.parts = []
		self.parts.append(Cylinder(world, space, 300., 0.2))
		self.parts.append(Cylinder(world, space, 300., 0.2))
		self.parts.append(Cylinder(world, space, 1000., 0.1))
		self.parts.append(Cylinder(world, space, 1000., 0.1))

		self.setPosition((0.,0.,0.))

		self.parts.append(Motor(self.parts[0], -20., self.parts[2]))
		self.parts.append(Motor(self.parts[1], -20., self.parts[2]))

		self.parts.append(Rod(self.parts[4], self.parts[2]))
		self.parts.append(Rod(self.parts[5], self.parts[2]))
		self.parts.append(Rod(self.parts[4], self.parts[3]))
		self.parts.append(Rod(self.parts[5], self.parts[3]))
		self.parts.append(Rod(self.parts[2], self.parts[3]))

	def Draw(self):
		for part in self.parts:
			part.Draw()

	def setPosition(self, pos):
		
		self.parts[0].setPosition(pos)
		self.parts[1].setPosition((pos[0]+1.,pos[1],pos[2]))
		self.parts[2].setPosition((pos[0]+0.51,pos[1]+0.45,pos[2]))
		self.parts[3].setPosition((pos[0]+0.51,pos[1]+0.15,pos[2]))

	def getPosition(self):
		return self.body1.getPosition()
		
	def UpdateInternalForces(self):
		for part in self.parts:
			part.UpdateInternalForces()

##########################################################

class Terrain:
	def __init__(self, world, space):

		self.verts = [(-100,0., -100.), (-100.,0.,100.), (100.,0.,0.)]
		self.faces = [(0,1,2)]

		self.meshdata = ode.TriMeshData()  #create the data buffer
		self.meshdata.build(self.verts, self.faces)  #Put vertex and face data into the buffer
		self.mesh = ode.GeomTriMesh(self.meshdata, space) #create collide mesh
		self.mesh.setPosition((0.,0.,0.))

	def Draw(self):
		#print self.body.getPosition()
		pass
		

##########################################################


# drop_object
def drop_object():
	"""Drop an object into the scene."""

	global bodies, geom, counter, objcount, objs

	obj = Composite(world, space, 1000, 0.1)

	obj.setPosition( (random.gauss(0,0.1),0.5,random.gauss(0,0.1)) )
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
	#print len(contacts)

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
#floor = ode.GeomPlane(space, (0,1,0), 0)
terrain = Terrain(world, space)

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

	terrain.Draw()
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
		if objcount==1:
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

		for obj in objs:
			obj.UpdateInternalForces()

		# Simulation step
		world.step(dt/n)

		# Remove all contact joints
		contactgroup.empty()

	lasttime = time.time()

glutIdleFunc (_idlefunc)

glutMainLoop ()
