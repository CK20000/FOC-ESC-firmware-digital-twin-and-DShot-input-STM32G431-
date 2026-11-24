import math

LUT_SIZE = 4096
FILENAME = "sin_lut.c"


def generate_lut():
    with open(FILENAME, "w") as f:
        f.write(f"// Auto-generated {LUT_SIZE}-point sine lookup table\n")
        f.write(f"// Maps index 0 -> {LUT_SIZE - 1} to 0 -> 2*PI\n\n")
        f.write(f"const float sin_lut[{LUT_SIZE}] = {{\n")

        for i in range(LUT_SIZE):
            # Calculate the angle in radians for this index
            angle = (i / LUT_SIZE) * 2.0 * math.pi
            sine_val = math.sin(angle)

            # Format to 7 decimal places with the 'f' suffix for C single-precision floats
            f.write(f"    {sine_val: .7f}f,")

            # Group into rows of 8 for readability
            if (i + 1) % 8 == 0:
                f.write("\n")

        f.write("};\n")
    print(f"Successfully generated {FILENAME} with {LUT_SIZE} points.")


if __name__ == "__main__":
    generate_lut()