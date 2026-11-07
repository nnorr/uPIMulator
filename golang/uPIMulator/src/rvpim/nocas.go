//go:build !rvpim

package rvpim

import (
	"fmt"
	"os"
	"uPIMulator/src/misc"
)

// Run is the default build's placeholder: the co-simulator needs cgo and libcas.a
// (go build -tags rvpim, see rvpim.go), so the stock build stays dependency-free.
func Run(*misc.CommandLineParser) int {
	fmt.Fprintln(os.Stderr, "rvpim: this binary was built without -tags rvpim (see src/rvpim/rvpim.go)")
	return 2
}
