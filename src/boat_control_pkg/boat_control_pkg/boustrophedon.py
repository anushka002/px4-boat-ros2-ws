def generate_boustrophedon_path(start_x, start_y, width, height, step):
    """
    Generate a simple boustrophedon (lawnmower) path starting from (start_x, start_y)
    covering a rectangle of given width and height with a given step size.
    """
    path = []
    rows = int(height / step)
    cols = int(width / step)

    y = start_y
    for i in range(rows + 1):
        if i % 2 == 0:
            # left to right
            x_range = range(cols + 1)
        else:
            # right to left
            x_range = reversed(range(cols + 1))

        for j in x_range:
            x = start_x + j * step
            path.append([x, y])

        y += step

    return path

