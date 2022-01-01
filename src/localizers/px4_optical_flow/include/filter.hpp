template<uint8_t N>
class SMA {
public:
    double operator()(double input) {
        sum -= previousInputs[index];
        sum += input;
        previousInputs[index] = input;

        n++;
        if (n > N) n = N;

        if (++index == N)
            index = 0;
        return sum / n;
    }

private:
    uint8_t index = 0;
    double previousInputs[N] = {0};
    double sum = 0;
    double n = 0;
};