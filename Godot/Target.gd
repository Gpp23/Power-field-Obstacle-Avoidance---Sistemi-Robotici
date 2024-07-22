extends Node3D

@export var udpPort: int = 4446

var server: UDPServer
var target : MeshInstance3D

# Called when the node enters the scene tree for the first time.
func _ready():
	#inizializzazione server
	server = UDPServer.new()
	server.listen(udpPort)
	
	target = $MeshInstance3D


func _physics_process(delta):
	#print(theRobot.global_position.x, " ", -theRobot.global_position.z, " ", theRobot.global_rotation.y)
	server.poll()
	
	if server.is_connection_available():
		var peer: PacketPeerUDP = server.take_connection()
		
		
		var tosend =  PackedFloat32Array()
		
		tosend.append(target.global_position.x) # x
		tosend.append(-target.global_position.z) # y
		
		peer.put_var(tosend)
		
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	pass

func _integrate_forces(state):
	pass
