package logic

import (
	"math/rand"
	"testing"
)

// The bitwise ALU ops against Go's operators on 32-bit operands (Xor and Nxor used to
// return operand2 and its complement).
func TestAluBitwise(t *testing.T) {
	alu := new(Alu)
	ops := map[string]struct {
		got  func(int64, int64) int64
		want func(uint32, uint32) uint32
	}{
		"and":  {alu.And, func(a, b uint32) uint32 { return a & b }},
		"nand": {alu.Nand, func(a, b uint32) uint32 { return ^(a & b) }},
		"andn": {alu.Andn, func(a, b uint32) uint32 { return ^a & b }},
		"or":   {alu.Or, func(a, b uint32) uint32 { return a | b }},
		"nor":  {alu.Nor, func(a, b uint32) uint32 { return ^(a | b) }},
		"orn":  {alu.Orn, func(a, b uint32) uint32 { return ^a | b }},
		"xor":  {alu.Xor, func(a, b uint32) uint32 { return a ^ b }},
		"nxor": {alu.Nxor, func(a, b uint32) uint32 { return ^(a ^ b) }},
	}
	r := rand.New(rand.NewSource(1))
	for name, op := range ops {
		for i := 0; i < 1000; i++ {
			a, b := r.Uint32(), r.Uint32()
			if got := uint32(op.got(int64(a), int64(b))); got != op.want(a, b) {
				t.Fatalf("%s(%#x, %#x) = %#x, want %#x", name, a, b, got, op.want(a, b))
			}
		}
	}
}
