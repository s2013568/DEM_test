# environments.py

environments = {
    "default": [
        (0, 0, 100, 0),  # Bottom wall from (0, 0) to (100, 0)
        (0, 2, 100, 2)   # Top wall from (0, 2) to (100, 2)
    ],
    "Straight Flow": [
        (0, 0, 100, 0),  # Bottom wall from (0, 0) to (100, 0)
        (0, 10, 100, 10)   # Top wall from (0, 10) to (100, 10)
    ],
    "Rounding Corner": [
        (0, 0, 50, 0),  
        (0, 10, 60, 10),
        (60, 10, 60, -100),
        (50, 0, 50, -100)
    ], 
    "Entering": [
        (50, 0, 50, 3),  
        (50, 7, 50, 10)
    ]
}
