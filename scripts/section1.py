#!/usr/bin/env python3

# add import and helper functions here
import numpy as np

if __name__ == "__main__":
    np.random.seed(42)
    A = np.random.normal(size=(4, 4))
    B = np.random.normal(size=(4, 2))

    array_mult = A @ B

    np.random.seed(42)
    x = np.random.normal(size=(4,10))

    x_reshape = x[None]
    x_normal = x[:, None]

    x_subtract = x_normal - x_reshape
    print(x_subtract.shape)
    distance = np.sum((x_subtract)**2, axis=-1)
    print("Array mult:", array_mult)
    print("Distance: ", distance)
