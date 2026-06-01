package main

import (
	"log/slog"

	"github.com/poisnoir/spine-go"
)

type Robot struct {
	name          string
	logger        *slog.Logger
	currentJoints [6]float64
	inputSource   *spine.Subscriber[[4][4]float64]
	outputSource  *spine.Publisher[[6]float64]
}

func NewRobot(name string, logger *slog.Logger, input *spine.Subscriber[[4][4]float64], output *spine.Publisher[[6]float64]) *Robot {
	return &Robot{
		name:          name,
		logger:        logger,
		inputSource:   input,
		outputSource:  output,
		currentJoints: [6]float64{0, 0, 0, 0, 0, 0},
	}
}

func (r *Robot) GetJoints() {

}

func (r *Robot) Run() {
	for {
		input, err := r.inputSource.Get()
		if err != nil {
			// err
			continue
		}
		joints := InverseKinematics(input)
		if len(joints) == 0 {
			// err
			continue
		}

		r.currentJoints = joints[0]
		r.outputSource.Publish(joints[0])
	}
}
