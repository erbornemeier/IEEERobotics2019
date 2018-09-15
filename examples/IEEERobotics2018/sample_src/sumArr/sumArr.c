
extern "C" {
    int getCenter(int* pts, int pts_size) {
        int black_pixels = 0,
            pixel_sums = 0;

        for (int i = 0; i < pts_size; i++) {
            if (pts[i] == 0) {
                black_pixels++;
                pixel_sums += i;
            }
        }

        if (black_pixels == 0)
            return -1;

        return pixel_sums / black_pixels;
    }
}
