# ROADEF2018-GuillotineCut
Guillotine Cut solver for ROADEF Challenge 2018 by team J29.
# Problem description:
The problem was proposed by ROADEF/EURO, and you can download the [Problem Description](http://www.roadef.org/challenge/2018/files/Challenge_ROADEF_EURO_SG_Description.pdf) in ROADEF official site.
# Build command:
> make
# Test command:
+ Test environment is Ubuntu 18.04.
## Param:
+ -t time_limit to stop the program execution after time_limit seconds (caution: this is "real time", not cputime).
+ -p instance_name to load the data associated with the instance.
+ -o new_solution_filename to designate the result file.
+ -name to return the teamId that is the author of the executable (if it is the only option the executable returns the team identifier and quits).
+ -s seed force program with random to be deterministic.
## Example:
> ./challengeSG -t 180 -p Instance/A1 -o Solution180/A1_solution.csv
