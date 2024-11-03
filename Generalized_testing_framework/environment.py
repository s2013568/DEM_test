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
    "wide_path": [
        (0, 0, 100, 0),  # Bottom wall from (0, 0) to (100, 0)
        (0, 3, 100, 3)   # Top wall at a higher position for a wider path
    ]
}
