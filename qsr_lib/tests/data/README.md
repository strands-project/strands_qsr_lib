### Datasets description

* `data1.csv`: A set of hand-drawn int 2D-points for 2 objects ("o1", "o2") with fixed width and length
(i.e. bounding boxes are provided). Tried to cover all possible cardir and RCC states.
* `data2.csv`: A set of 10K random integer 2D-points (`random.randint(0, 50)`) for 3 objects ("o1", "o2", "o3")
and random float width and length (`random.randint(3, 6)`) (i.e. bounding boxes are provided).
* `data3.csv`: A set of 10K random integer 2D-points (`random.randint(0, 50)`) for 3 objects ("o1", "o2", "o3"),
 but no width and length are given (i.e. no bounding boxes are provided).
* `data4.csv`: A set of 10K random float 2D-points (`random.uniform(0, 50)`) for 3 objects ("o1", "o2", "o3")
and random float width and length (`random.randint(3, 6)`) (i.e. bounding boxes are provided).
* `data2_first100.csv`: First 100 instances of data2.
* `data3_first100.csv`: First 100 instances of data3.
* `data4_first100.csv`: First 100 instances of data4.

### Running the tests

```bash
rosrun qsr_lib unittests_generate_ground_truth.py -i <world name> -o <output filename> <qsr>
```

E.g.
```bash
rosrun qsr_lib unittests_generate_ground_truth.py -i data1 -o qsr_lib/tests/data/data1_rcc4_defaults.txt rcc4
```

`-i` options: data1 | data2 | data3 | data4

#### Types of generated data

* Defaults. E.g.
```bash
rosrun qsr_lib unittests_generate_ground_truth.py -i data1 -o qsr_lib/tests/data/data1_rcc4_defaults.txt rcc4
```
