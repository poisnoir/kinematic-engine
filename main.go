package main

import "fmt"

func main() {

	HOME_MATRIX := ForwardKinematics([6]float64{0, 0, 0, 0, 0, 0})

	fmt.Println(HOME_MATRIX)
}
