#ifndef QUICKSELECT_H
#define QUICKSELECT_H

int partition(std::vector<float>& vec, int left, int right, int pivotIndex) {
    int pivotValue = vec[pivotIndex];
    std::swap(vec[pivotIndex], vec[right]);  // Move pivot to end
    int storeIndex = left;
    for (int i = left; i < right; i++) {
        if (vec[i] < pivotValue) {
            std::swap(vec[storeIndex], vec[i]);
            storeIndex++;
        }
    }
    std::swap(vec[right], vec[storeIndex]);  // Move pivot to its final place
    return storeIndex;
}

int quickselect(std::vector<float>& vec, int left, int right, int k) {
    if (left == right) {
        return vec[left];
    }

    int pivotIndex = left + rand() % (right - left + 1);
    pivotIndex = partition(vec, left, right, pivotIndex);

    if (k == pivotIndex) {
        return vec[k];
    } else if (k < pivotIndex) {
        return quickselect(vec, left, pivotIndex - 1, k);
    } else {
        return quickselect(vec, pivotIndex + 1, right, k);
    }
}

double getMedian(std::vector<float>& vec) {
    int n = vec.size();
    if (n == 0) {
        return 0.0;  // Handle empty vector
    }

    if (n % 2 == 1) {
        return quickselect(vec, 0, n - 1, n / 2);
    } else {
        return 0.5 * (quickselect(vec, 0, n - 1, n / 2 - 1) + quickselect(vec, 0, n - 1, n / 2));
    }
}

int partition(std::vector<int>& vec, int left, int right, int pivotIndex) {
    int pivotValue = vec[pivotIndex];
    std::swap(vec[pivotIndex], vec[right]);  // Move pivot to end
    int storeIndex = left;
    for (int i = left; i < right; i++) {
        if (vec[i] < pivotValue) {
            std::swap(vec[storeIndex], vec[i]);
            storeIndex++;
        }
    }
    std::swap(vec[right], vec[storeIndex]);  // Move pivot to its final place
    return storeIndex;
}

int quickselect(std::vector<int>& vec, int left, int right, int k) {
    if (left == right) {
        return vec[left];
    }

    int pivotIndex = left + rand() % (right - left + 1);
    pivotIndex = partition(vec, left, right, pivotIndex);

    if (k == pivotIndex) {
        return vec[k];
    } else if (k < pivotIndex) {
        return quickselect(vec, left, pivotIndex - 1, k);
    } else {
        return quickselect(vec, pivotIndex + 1, right, k);
    }
}

double getMedian(std::vector<int>& vec) {
    int n = vec.size();
    if (n == 0) {
        return 0.0;  // Handle empty vector
    }

    if (n % 2 == 1) {
        return quickselect(vec, 0, n - 1, n / 2);
    } else {
        return 0.5 * (quickselect(vec, 0, n - 1, n / 2 - 1) + quickselect(vec, 0, n - 1, n / 2));
    }
}

#endif
