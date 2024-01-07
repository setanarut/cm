//go:build debug

package cm

import "fmt"

func assert(truth bool, msg ...interface{}) {
	if !truth {
		panic(fmt.Sprint("Assertion failed: ", msg))
	}
}
