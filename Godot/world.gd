extends Node3D

# Parametri
var num_obstacles = 3
var min_distance = 1.5  # Distanza minima tra gli ostacoli
var min_value = -2.0
var max_value = 1.6
var max_attempts = 100  # Numero massimo di tentativi per trovare una posizione valida
var default_positions = [Vector2(-1, 2), Vector2(1, -2), Vector2(1, 1)]  # Posizioni di default per i fallimenti

func _ready():
	var obstacle_positions = generate_obstacle_positions(num_obstacles, min_distance, min_value, max_value)
	var index = ['A', 'B', 'C']
	# Imposta le posizioni degli ostacoli
	for i in range(num_obstacles):
		var obstacle = get_node("Obstacles/obstacle" + index[i])
		
		obstacle.global_position.x = obstacle_positions[i].x
		obstacle.global_position.z = obstacle_positions[i].y
		print("Obstacle ", i + 1, " position: x = ", obstacle_positions[i].x, ", z = ", obstacle_positions[i].y)

# Genera una posizione casuale all'interno dell'intervallo specificato per un singolo asse
func generate_random_position_axis(min_value: float, max_value: float) -> float:
	return randf_range(min_value, max_value)

# Verifica se una nuova posizione è valida (cioè, non collidono con le posizioni degli ostacoli esistenti)
func is_valid_position(new_pos: Vector2, existing_positions: Array, min_distance: float) -> bool:
	for pos in existing_positions:
		if new_pos.distance_to(pos) < min_distance:
			return false
	return true

# Genera posizioni per un numero specificato di ostacoli, assicurandosi che non collidano tra loro e non siano allineati diagonalmente
func generate_obstacle_positions(num_obstacles: int, min_distance: float, min_value: float, max_value: float) -> Array:
	var positions = []
	var range_size_x = (max_value - min_value) / num_obstacles
	var range_size_z = (max_value - min_value) / num_obstacles
	
	for i in range(num_obstacles):
		var range_min_x = min_value + i * range_size_x
		var range_max_x = range_min_x + range_size_x
		var range_min_z = min_value + i * range_size_z
		var range_max_z = range_min_z + range_size_z
		var attempts = 0
		var found = false
		
		while attempts < max_attempts:
			var new_pos = Vector2(generate_random_position_axis(range_min_x, range_max_x), generate_random_position_axis(range_min_z, range_max_z))
			if is_valid_position(new_pos, positions, min_distance):
				positions.append(new_pos)
				found = true
				break
			attempts += 1
		
		if not found:
			print("Failed to place obstacle ", i + 1, " after ", max_attempts, " attempts.")
			# Aggiungi una posizione di default se fallisce
			if i < default_positions.size():
				positions.append(default_positions[i])
			else:
				# Se non ci sono abbastanza posizioni di default, aggiungi una posizione arbitraria
				positions.append(Vector2(0, 0))
	
	return positions
