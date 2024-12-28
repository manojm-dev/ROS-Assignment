import sqlite3

# Function to retrieve goal data by name
def get_goal_by_name(name):
    # Connect to the SQLite database
    conn = sqlite3.connect('goals.db')
    cursor = conn.cursor()
    
    # Execute SQL query to fetch the goal data for the given name
    cursor.execute('''
    SELECT position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w
    FROM goals WHERE name = ?
    ''', (name,))
    
    # Fetch the result
    row = cursor.fetchone()
    
    # Close the connection
    conn.close()
    
    if row:
        # If the goal is found, format the data into a dictionary
        goal = {
            "name": name,
            "position": {"x": row[0], "y": row[1], "z": row[2]},
            "orientation": {"x": row[3], "y": row[4], "z": row[5], "w": row[6]}
        }
        return goal
    else:
        return None

# Fetch the "home" goal data
home_goal = get_goal_by_name("home")


# Fetch the "table1" goal data
table1_goal = get_goal_by_name("table1")


# Fetch the "table2" goal data
table2_goal = get_goal_by_name("table2")


# Fetch the "table3" goal data
table3_goal = get_goal_by_name("table3")


# Fetch the "kitchen" goal data
kitchen_goal = get_goal_by_name("kitchen")

