import glob

read_files = glob.glob("results_dyna_5_*.csv")

with open("results_dyna_5.csv", "wb") as outfile:
    for f in read_files:
        with open(f, "rb") as infile:
            outfile.write(infile.read())