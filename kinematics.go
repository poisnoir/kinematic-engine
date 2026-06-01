package main

import (
	"math"

	"gonum.org/v1/gonum/mat"
)

// Todo: I am using memory crazy in this code without reuse. If there is a lag in future this code can be optimized

func InverseKinematics(transform [4][4]float64) [][6]float64 {
	return nil
}

func ForwardKinematics(joints [6]float64) [4][4]float64 {
	result := mat.NewDense(4, 4, nil)
	result.Product(q1_transform(joints[0]), q2_transform(joints[1]), q3_transform(joints[2]), q4_transform(joints[3]), q5_transform(joints[4]), q6_transform(joints[5]))

	var out [4][4]float64

	for r := 0; r < 4; r++ {
		for c := 0; c < 4; c++ {
			out[r][c] = result.At(r, c)
		}
	}

	return out

}

// ********************************************************************************************
// Below code is hardcoded functions that return arctos arm joints forward kinematics functions
// q1_transform: Alpha=0, a=0, theta=0, d=287.87
func q1_transform(q float64) *mat.Dense {
	q_sin := math.Sin(q)
	q_cos := math.Cos(q)

	return mat.NewDense(4, 4, []float64{
		q_cos, -q_sin, 0, 0,
		q_sin, q_cos, 0, 0,
		0, 0, 1, 287.87,
		0, 0, 0, 1,
	})
}

// q2_transform: Alpha=-90, a=20.174, theta=-90, d=0
func q2_transform(q float64) *mat.Dense {
	q_sin := math.Sin(q)
	q_cos := math.Cos(q)

	return mat.NewDense(4, 4, []float64{
		q_sin, 0, -q_cos, 20.174 * q_sin,
		-q_cos, 0, -q_sin, -20.174 * q_cos,
		0, -1, 0, 0,
		0, 0, 0, 1,
	})
}

// q3_transform: Alpha=0, a=260.986, theta=0, d=0
func q3_transform(q float64) *mat.Dense {
	q_sin := math.Sin(q)
	q_cos := math.Cos(q)

	return mat.NewDense(4, 4, []float64{
		q_cos, -q_sin, 0, 260.986 * q_cos,
		q_sin, q_cos, 0, 260.986 * q_sin,
		0, 0, 1, 0,
		0, 0, 0, 1,
	})
}

// q4_transform: Alpha=0, a=19.219, theta=0, d=260.753
func q4_transform(q float64) *mat.Dense {
	q_sin := math.Sin(q)
	q_cos := math.Cos(q)

	return mat.NewDense(4, 4, []float64{
		q_cos, -q_sin, 0, 19.219 * q_cos,
		q_sin, q_cos, 0, 19.219 * q_sin,
		0, 0, 1, 260.753,
		0, 0, 0, 1,
	})
}

// q5_transform: Alpha=90, a=0, theta=0, d=0
func q5_transform(q float64) *mat.Dense {
	q_sin := math.Sin(q)
	q_cos := math.Cos(q)

	return mat.NewDense(4, 4, []float64{
		q_cos, 0, q_sin, 0,
		q_sin, 0, -q_cos, 0,
		0, 1, 0, 0,
		0, 0, 0, 1,
	})
}

// q6_transform: Alpha=-90, a=0, theta=180, d=74.745
func q6_transform(q float64) *mat.Dense {
	q_sin := math.Sin(q)
	q_cos := math.Cos(q)

	return mat.NewDense(4, 4, []float64{
		-q_cos, 0, q_sin, 0,
		-q_sin, 0, -q_cos, 0,
		0, -1, 0, 74.745,
		0, 0, 0, 1,
	})
}
