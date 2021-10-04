[Create a ChArUco board image]
Usage: create_board_charuco [params] outfile 

	--bb (value:1)
		Number of bits in marker borders
	-d
		dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16
	-h
		Number of squares in Y direction
	-m
		Margins size (in pixels). Default is (squareLength-markerLength)
	--ml
		Marker side length (in pixels)
	--si (value:false)
		show generated image
	--sl
		Square side length (in pixels)
	-w
		Number of squares in X direction

	outfile (value:<none>)
		Output image

example: ./create_board_charuco ./charuco.png -w=5 -h=7 -sl=100 -ml=50 -d=0

------------------------------------------------------------------------------------------

[Create an ArUco grid board image]
Usage: create_board [params] outfile 

	--bb (value:1)
		Number of bits in marker borders
	-d
		dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16
	-h
		Number of markers in Y direction
	-l
		Marker side length (in pixels)
	-m
		Margins size (in pixels). Default is marker separation (-s)
	-s
		Separation between two consecutive markers in the grid (in pixels)
	--si (value:false)
		show generated image
	-w
		Number of markers in X direction

	outfile (value:<none>)
		Output image

example: ./create_board ./aruco.png -w=5 -h=7 -l=100 -s=10 -d=10
