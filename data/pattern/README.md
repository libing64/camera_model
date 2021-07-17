# Create your own pattern
Now, if you want to create your own pattern, you will need python to use https://github.com/opencv/opencv/blob/master/doc/pattern_tools/gen_pattern.py

Example

create a checkerboard pattern in file chessboard.svg with 9 rows, 6 columns and a square size of 20mm:
```
python gen_pattern.py -o chessboard.svg --rows 10 --columns 7 --type checkerboard --square_size 25
```


create a circle board pattern in file circleboard.svg with 9 rows, 6 columns and a radius of 15mm:
```
python gen_pattern.py -o circleboard.svg --rows 9 --columns 6 --type circles --square_size 30 --radius_rate 4.0

```

create a circle board pattern in file acircleboard.svg with 7 rows, 5 columns and a square size of 10mm and less spacing between circle:

```
python gen_pattern.py -o acircleboard.svg --rows 11 --columns 4 --type acircles --square_size 30 --radius_rate 4
```

If you want to change unit use -u option (mm inches, px, m)

If you want to change page size use -w and -h options

If you want to create a ChArUco board read tutorial Detection of ChArUco Corners in opencv_contrib tutorial.