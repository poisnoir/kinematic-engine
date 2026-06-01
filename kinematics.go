package main

import (
	"math"

	"gonum.org/v1/gonum/mat"
)

// Todo: I am using memory crazy in this code without reuse. If there is a lag in future this code can be optimized

// This matrix decouples q5 and q6 from target end point resulting in creating position for q1..4 joints
var q5q6Remove *mat.Dense = mat.NewDense(4, 4, []float64{
	-1, 0, 0, 0,
	0, 1, 0, 74.745,
	0, 0, 1, 0,
	0, 0, 0, 1,
})

func InverseKinematics(transform [4][4]float64) [][6]float64 {

	// transform[2, 1] = -cos(q3 + q5)
	// transform[2, 2] = sin(q₆)⋅sin(q₃ + q₅)

	var sum_q3q5 [2]float64
	sum_q3q5[0] = math.Acos(-transform[2][1])
	sum_q3q5[1] = -sum_q3q5[0]

	for _, sum_q3q5 := range sum_q3q5 {

		sin_q6 := transform[2][2] / math.Sin(sum_q3q5)
		var q6s [2]float64
		q6s[0] = math.Asin(sin_q6)
		q6s[1] = math.Pi - q6s[0]

		// transform[2, 3] = -280.205⋅sin(q₃) + 74.745⋅cos(q₃ + q₅) + 287.87
		sin_q3 := (transform[2][3] - 287.87 + transform[2][1]*74.745) / -280.205

		var q3s [2]float64
		q3s[0] = math.Asin(sin_q3)
		q3s[1] = math.Pi - q3s[0]

		var q5s [2]float64
		q5s[0] = sum_q3q5 - q3s[0]
		q5s[1] = sum_q3q5 - q3s[1]

	}

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
	s, c := math.Sincos(q)
	return mat.NewDense(4, 4, []float64{
		c, -s, 0, 0,
		s, c, 0, 0,
		0, 0, 1, 287.87,
		0, 0, 0, 1,
	})
}

// q2_transform: Alpha=-90, a=20.174, theta=-90, d=0
func q2_transform(q float64) *mat.Dense {
	s, c := math.Sincos(q)
	return mat.NewDense(4, 4, []float64{
		c, 0, -s, 20.174 * c,
		s, 0, c, 20.174 * s,
		0, -1, 0, 0,
		0, 0, 0, 1,
	})
}

// q3_transform: Alpha=0, a=260.986, theta=0, d=0
func q3_transform(q float64) *mat.Dense {
	s, c := math.Sincos(q)
	return mat.NewDense(4, 4, []float64{
		c, -s, 0, 260.986 * c,
		s, c, 0, 260.986 * s,
		0, 0, 1, 0,
		0, 0, 0, 1,
	})
}

// q4_transform: Alpha=0, a=19.219, theta=0, d=260.753
func q4_transform(q float64) *mat.Dense {
	s, c := math.Sincos(q)
	return mat.NewDense(4, 4, []float64{
		c, -s, 0, 19.219 * c,
		s, c, 0, 19.219 * s,
		0, 0, 1, 260.753,
		0, 0, 0, 1,
	})
}

// q5_transform: Alpha=90, a=0, theta=0, d=0
func q5_transform(q float64) *mat.Dense {
	s, c := math.Sincos(q)
	return mat.NewDense(4, 4, []float64{
		c, 0, s, 0,
		s, 0, -c, 0,
		0, 1, 0, 0,
		0, 0, 0, 1,
	})
}

// q6_transform: Alpha=-90, a=0, theta=180, d=74.745
func q6_transform(q float64) *mat.Dense {
	theta := q + math.Pi
	s, c := math.Sincos(theta)
	return mat.NewDense(4, 4, []float64{
		c, 0, s, 0,
		s, 0, -c, 0,
		0, -1, 0, 74.745,
		0, 0, 0, 1,
	})
}
