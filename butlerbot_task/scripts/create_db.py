import sqlite3

# Create and connect to SQLite database (it will create the file if it doesn't exist)
conn = sqlite3.connect('goals.db')
cursor = conn.cursor()

# Create table to store goals
cursor.execute('''
CREATE TABLE IF NOT EXISTS goals (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT,
    position_x REAL,
    position_y REAL,
    position_z REAL,
    orientation_x REAL,
    orientation_y REAL,
    orientation_z REAL,
    orientation_w REAL
)
''')

# Function to insert a goal into the database
def insert_goal(name, position, orientation):
    cursor.execute('''
    INSERT INTO goals (name, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)
    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
    ''', (name, position['x'], position['y'], position['z'], orientation['x'], orientation['y'], orientation['z'], orientation['w']))
    conn.commit()

# Insert the provided goals into the database
goals = [
    {
        "name": "table1",
        "position": {"x": 2.5358612728358585, "y": -4.257302366795823, "z": 0.010028244526066134},
        "orientation": {"x": 0.00018268282181411002, "y": 0.0004111581670754367, "z": 0.8026696125007791, "w": 0.5964237509890074}
    },
    {
        "name": "table2",
        "position": {"x": 5.601735197265228, "y": -4.198969803526397, "z": 0.010029875041603},
        "orientation": {"x": 0.00018130474813125575, "y": 0.0004003978439136958, "z": 0.7842191769200315, "w": 0.6204837543087033}
    },
    {
        "name": "table3",
        "position": {"x": 8.36097011687938, "y": -4.360378002879507, "z": 0.010019767275631891},
        "orientation": {"x": -0.000269246437016163, "y": -0.0002874608713562783, "z": -0.6197654483566257, "w": -0.7847869990614742}
    },
    {
        "name": "home",
        "position": {"x": 3.9999010226286433, "y": -9.400774414704287, "z": 0.010026648956927495},
        "orientation": {"x": 0.00023148211747490028, "y": 0.00035445552609365805, "z": 0.6816773986567048, "w": 0.7316527488760891}
    },
    {
        "name": "kitchen",
        "position": {"x": 11.480972204854208, "y": -7.025651170323608, "z": 0.01001839234929796},
        "orientation": {"x": -0.000325124282815795, "y": 0.00033219832823792015, "z": 0.78962982686241, "w": -0.6135833443532444}
    }
]

# Insert all goals into the database
for goal in goals:
    insert_goal(goal["name"], goal["position"], goal["orientation"])

# Commit and close the connection
conn.commit()
conn.close()
