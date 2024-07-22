extends Node3D

@export var udpPort: int = 4445

var server: UDPServer
var obstacleA : CollisionShape3D
var obstacleB : CollisionShape3D
var obstacleC : CollisionShape3D
#var test : MeshInstance3D

# Called when the node enters the scene tree for the first time.
func _ready():
	#inizializzazione server
	server = UDPServer.new()
	server.listen(udpPort)
	
	obstacleA = $obstacleA/CollisionShape3D
	obstacleB = $obstacleB/CollisionShape3D
	obstacleC = $obstacleC/CollisionShape3D


func _physics_process(delta):
	#print(theRobot.global_position.x, " ", -theRobot.global_position.z, " ", theRobot.global_rotation.y)
	server.poll()
	
	if server.is_connection_available():
		var peer: PacketPeerUDP = server.take_connection()
		
		
		var tosend =  PackedFloat32Array()
		
		tosend.append(obstacleA.global_position.x) # x
		tosend.append(-obstacleA.global_position.z) # y
		tosend.append(obstacleA.shape.radius)
		
		
		tosend.append(obstacleB.global_position.x) # x
		tosend.append(-obstacleB.global_position.z) # y
		tosend.append(obstacleB.shape.radius)
		
		tosend.append(obstacleC.global_position.x) # x
		tosend.append(-obstacleC.global_position.z) # y
		tosend.append(obstacleC.shape.radius)
		
		peer.put_var(tosend)
		
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	pass

func _integrate_forces(state):
	pass
