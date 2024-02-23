import numpy as np

cm_samples = [93.7, 91.8, 93.2, 92.3, 97.9, 96.0, 91.8, 94.2, 91.3, 98.4, 93.7, 97.0, 94.6, 93.2, 98.8, 94.2, 93.7, 90.4, 95.1, 96.0, 97.4, 93.2, 92.7, 100.3, 94.2, 96.5, 91.8, 94.2, 98.8, 93.2, 94.6, 93.7, 97.9, 99.3, 92.3, 94.2, 97.0, 92.3, 97.0, 94.2, 94.2, 95.1, 97.9, 99.8, 96.5, 96.0, 98.8, 99.3, 91.8, 97.0, 94.6, 98.4, 101.7, 97.9, 95.1, 96.0, 89.9, 89.9, 90.4, 94.6, 94.6, 91.8, 91.3, 92.3, 96.0, 92.3, 92.3, 96.0, 94.2, 94.6, 92.7, 93.2, 93.7, 97.4, 94.2, 94.6, 101.2, 94.6, 95.6, 96.0, 97.4, 96.5, 92.7, 92.7, 91.8, 94.6, 93.2, 97.0, 94.6, 99.3, 97.9, 96.0, 97.9, 97.4, 98.8, 92.7, 91.3, 91.3, 90.9, 94.2, 94.6, 96.0, 95.1, 93.2, 95.1, 94.6, 97.0, 93.7, 97.0, 93.7, 94.6, 94.6, 97.0, 95.6, 95.1, 92.7, 95.6, 91.8, 97.4, 97.4, 92.3, 92.7, 94.2, 96.0, 97.4, 95.1, 98.8, 98.4, 96.5, 96.0, 96.5, 96.0, 96.5, 95.6, 94.6, 99.3, 100.3, 96.0, 98.4, 97.4, 97.0, 96.5, 97.9, 95.6, 95.6, 97.9, 92.3, 96.5, 95.6, 95.6, 98.8, 98.4, 99.3, 93.7, 102.1, 95.6, 96.5, 101.7, 103.1, 107.8, 104.5, 109.6, 106.8, 105.4, 107.8, 109.2, 109.2, 108.2, 108.7, 108.7, 101.7, 100.3, 105.9, 104.5, 101.7, 99.3, 94.6, 97.0, 94.2, 95.6, 94.6, 93.7, 92.7, 92.0, 89.9, 89.9, 94.6, 98.8, 96.0, 97.9, 99.3, 95.1, 95.1, 94.6, 94.6, 91.1, 95.1, 95.6, 92.7, 92.0, 93.7, 91.3, 94.2, 93.7, 95.6, 94.2, 90.4, 90.4, 96.5, 95.1, 90.4, 92.0, 92.3, 96.0, 95.1, 93.2, 94.2, 97.0, 93.7, 92.7, 97.4, 94.2, 95.6, 96.5, 96.5, 93.2, 92.7, 93.7, 92.7, 93.2, 99.8, 95.1, 95.6, 96.0, 92.3, 96.5, 97.4, 97.9, 95.1, 93.2, 95.6, 93.7, 95.1, 97.9, 92.3, 95.1, 91.8, 94.2, 91.3, 93.7, 96.5, 94.2, 95.1, 90.9, 93.7, 93.7, 98.4, 96.0, 91.3, 90.9, 96.5, 96.5, 95.6, 94.6, 96.0, 92.7, 95.6, 95.1, 94.6, 92.3, 101.7, 95.6, 93.7, 94.6, 100.3, 99.3, 91.1, 98.4, 97.9, 100.3, 95.1, 98.8, 94.2, 93.7, 94.2, 96.0, 95.6, 91.8, 93.2, 96.0, 94.2, 94.2, 90.9, 90.4, 91.8, 92.3, 93.7, 89.2, 93.7, 92.3, 90.4, 93.2, 92.0, 88.7, 91.3, 92.0, 92.0, 88.7, 87.8, 91.5, 89.9, 89.9, 86.4, 90.9, 91.5, 87.3, 89.9, 94.2, 85.9, 88.7, 90.1, 94.2, 91.5, 95.6, 95.6, 91.1, 96.0, 91.3, 86.8, 96.5, 90.1, 98.8, 94.2, 91.3, 94.6, 90.1, 91.3, 94.6, 87.3, 98.8, 94.2, 84.5, 85.9, 90.9, 97.4, 90.6, 90.9, 94.6, 97.9, 98.4, 90.4, 92.7, 91.8, 92.0, 96.0, 97.9, 90.1, 90.9, 95.6, 95.6, 87.8, 87.8, 96.0, 96.0, 91.1, 91.1, 91.8, 91.8, 97.0, 97.0, 94.6, 94.6, 110.6, 110.6, 90.4, 90.4, 101.2, 101.2, 95.1, 95.1, 98.4, 98.4, 92.7, 92.7, 89.2, 86.8, 90.4, 90.6, 83.6, 90.4, 94.6, 89.7, 100.3, 95.6, 92.7, 91.5, 88.2, 87.3, 89.7, 93.7, 91.8, 84.0, 93.2, 87.8, 89.9, 92.7, 93.7, 85.0, 90.9, 90.1, 97.9, 89.9, 89.9, 97.9, 92.0, 90.9, 92.0, 92.0, 91.5, 90.4, 92.7, 93.2, 89.9, 92.0, 90.1, 95.1, 89.2, 85.4, 88.7, 86.8, 88.2, 94.6, 95.6, 82.1, 83.1, 91.1, 103.1, 90.6, 90.4, 94.2, 89.2, 91.3, 92.3, 102.1, 98.4, 95.6, 93.2, 90.6, 87.8, 86.4, 83.1, 93.2, 87.3, 87.8, 86.8, 91.8, 92.0, 95.6, 91.8, 83.1, 90.9, 89.2, 85.9, 90.1, 83.6, 83.1, 85.0, 85.0, 85.9, 85.0, 89.2, 92.7, 82.6, 92.3, 91.8, 90.1, 91.8, 79.3, 88.2, 82.1, 82.1, 90.9, 91.1, 94.2, 94.2, 90.1, 92.3, 83.1, 90.6, 85.0, 81.2, 87.8, 91.8, 84.0, 91.1, 86.4, 85.9, 92.0, 85.4, 94.6, 90.4, 90.1, 92.7, 85.9, 88.2, 82.6, 86.4, 92.3, 92.0, 91.8, 99.3, 89.2, 90.1, 88.2, 89.2, 91.8, 94.6, 91.1, 92.3, 88.2, 90.6, 91.5, 86.4, 90.9, 97.4, 96.5, 90.4, 82.6, 85.0, 87.3, 87.3, 98.4, 91.5, 91.1, 90.4, 90.6, 94.6, 94.6, 87.8, 87.8, 90.6, 85.9, 86.4, 88.7, 90.9, 90.4, 90.6, 91.3, 89.7, 89.2, 90.6, 90.6, 93.2, 93.2, 90.9, 90.9, 91.5, 92.0, 90.1, 88.2, 90.1, 89.2, 88.2, 85.0, 82.1, 85.0, 87.8, 92.3, 91.3, 96.5, 91.8, 94.2, 93.2, 92.0, 89.2, 91.5, 92.3, 89.9, 94.6, 92.7, 91.3, 93.2, 96.0, 97.0, 100.3, 96.5, 99.8, 95.6, 92.3, 94.2, 89.5, 91.5, 94.6, 90.6, 89.9, 91.3, 89.2, 89.5, 91.5, 91.1, 91.3, 91.5, 94.2, 89.9, 92.3, 92.3, 91.3, 94.2, 95.6, 91.8, 97.4, 94.6, 91.8, 92.3, 95.6, 92.3, 89.5, 97.9, 97.9, 94.2, 94.2, 90.4, 90.4, 90.4, 90.4, 98.4, 98.4, 104.5, 104.5, 98.4, 99.3, 93.7, 91.3, 99.8, 94.2, 91.3, 92.7, 90.4, 90.4, 95.6, 92.3, 91.8, 95.1, 91.3, 95.6, 90.1, 91.5, 95.6, 90.1, 97.4, 96.0, 92.3, 98.4, 96.0, 92.3, 96.0, 95.6, 103.5, 98.4, 104.0, 104.0, 95.6, 100.3, 95.1, 97.0, 99.3, 92.7, 100.3, 103.5, 92.7, 103.1, 91.1, 87.8, 91.5, 89.5, 103.5, 96.5, 96.0, 101.2, 86.8, 97.0, 91.5, 89.2, 91.5, 88.7, 96.5, 93.2, 91.3, 96.0, 90.1, 83.1, 89.9, 88.7, 98.8, 89.9, 91.3, 91.8, 91.8, 90.1, 90.1, 97.0, 92.7, 95.6, 92.7, 89.9, 92.3, 92.3, 93.2, 93.2, 89.5, 89.5, 91.5, 91.5, 90.4, 90.4, 91.3, 91.3, 98.4, 98.4, 103.5, 103.5, 97.0, 97.0, 92.3, 92.3, 87.8, 87.8, 94.6, 94.6, 85.4, 85.4, 89.9, 89.9, 101.2, 101.2, 90.6, 90.6, 104.9, 104.9, 97.9, 94.2, 100.7, 93.7, 92.3, 95.1, 97.9, 94.2, 99.3, 97.0, 90.9, 100.3, 100.7, 97.0, 96.5, 98.4, 92.7, 100.7, 94.6, 101.7, 95.6, 96.5, 93.2, 101.2, 96.5, 92.0, 88.2, 85.4, 93.7, 85.9, 90.6, 90.9, 92.0, 95.6, 90.4, 95.1, 95.1, 93.7, 100.3, 89.2, 91.3, 89.2, 99.3, 92.7, 95.6, 99.3, 100.3, 103.1, 89.9, 92.3, 89.9, 99.3, 89.5, 91.5, 92.0, 91.8, 93.7, 88.7, 89.5, 90.9, 92.7, 95.1, 92.7, 92.0, 87.8, 91.5, 90.1, 88.7, 101.2, 94.2, 86.8, 86.8, 88.7, 85.4, 85.9, 91.3, 98.8, 89.5, 87.8, 90.6, 90.6, 89.2, 92.7, 86.4, 85.0, 87.3, 93.7, 88.2, 88.7, 91.5, 89.2, 89.5, 86.4, 85.9, 89.2, 85.0, 87.3, 80.7, 88.7, 85.4, 82.6, 87.8, 90.4, 88.2, 89.2, 86.4, 85.9, 89.2, 91.8, 84.5, 85.9, 86.8, 88.2, 88.2, 89.7, 86.4, 88.2, 88.7, 89.9, 91.3, 86.4, 84.0, 86.4, 84.0, 86.8, 89.2, 87.3, 87.3, 85.9, 90.1, 92.3, 84.0, 91.5, 85.4, 85.0, 89.7, 91.5, 89.2, 89.9, 90.6, 89.9, 91.1, 89.7, 89.5, 92.7, 89.5, 90.4, 88.7, 91.8, 92.0, 90.1, 90.6, 90.9, 92.7, 89.5, 89.5, 91.8, 90.9, 91.3, 93.7, 91.3, 91.3, 92.7, 91.5, 89.5, 92.0, 92.3, 91.5, 89.9, 92.0, 90.4, 90.9, 92.0, 90.6, 92.7, 91.3, 90.4, 92.7, 94.6, 91.3, 93.2, 92.7, 92.7, 92.7, 95.6, 92.7, 90.4, 89.5, 90.9, 92.3, 90.4, 89.9, 91.5, 91.3, 89.5, 93.7, 89.5, 91.5, 91.8, 92.3, 89.5, 93.2, 93.7, 93.2, 94.6, 92.3, 93.2, 92.7, 95.1, 94.2, 91.3, 89.5, 91.3, 93.2, 89.9, 91.3, 89.9, 92.3, 92.0, 93.7, 95.6, 90.4, 92.3, 92.0, 90.9, 91.8, 91.3, 92.7, 92.3, 92.3, 90.1, 92.0, 92.0, 91.3, 94.2, 90.9, 93.7, 90.9, 88.7, 93.2, 93.2, 90.4, 90.4, 91.8, 92.3, 89.5, 92.7, 89.2, 89.2, 91.5, 92.3, 91.3, 89.7, 85.9, 90.6, 89.5]
m_samples = [0.94, 0.92, 0.93, 0.92, 0.98, 0.96, 0.92, 0.94, 0.91, 0.98, 0.94, 0.97, 0.95, 0.93, 0.99, 0.94, 0.94, 0.9, 0.95, 0.96, 0.97, 0.93, 0.93, 1.0, 0.94, 0.96, 0.92, 0.94, 0.99, 0.93, 0.95, 0.94, 0.98, 0.99, 0.92, 0.94, 0.97, 0.92, 0.97, 0.94, 0.94, 0.95, 0.98, 1.0, 0.96, 0.96, 0.99, 0.99, 0.92, 0.97, 0.95, 0.98, 1.02, 0.98, 0.95, 0.96, 0.9, 0.9, 0.9, 0.95, 0.95, 0.92, 0.91, 0.92, 0.96, 0.92, 0.92, 0.96, 0.94, 0.95, 0.93, 0.93, 0.94, 0.97, 0.94, 0.95, 1.01, 0.95, 0.96, 0.96, 0.97, 0.96, 0.93, 0.93, 0.92, 0.95, 0.93, 0.97, 0.95, 0.99, 0.98, 0.96, 0.98, 0.97, 0.99, 0.93, 0.91, 0.91, 0.91, 0.94, 0.95, 0.96, 0.95, 0.93, 0.95, 0.95, 0.97, 0.94, 0.97, 0.94, 0.95, 0.95, 0.97, 0.96, 0.95, 0.93, 0.96, 0.92, 0.97, 0.97, 0.92, 0.93, 0.94, 0.96, 0.97, 0.95, 0.99, 0.98, 0.96, 0.96, 0.96, 0.96, 0.96, 0.96, 0.95, 0.99, 1.0, 0.96, 0.98, 0.97, 0.97, 0.96, 0.98, 0.96, 0.96, 0.98, 0.92, 0.96, 0.96, 0.96, 0.99, 0.98, 0.99, 0.94, 1.02, 0.96, 0.96, 1.02, 1.03, 1.08, 1.04, 1.1, 1.07, 1.05, 1.08, 1.09, 1.09, 1.08, 1.09, 1.09, 1.02, 1.0, 1.06, 1.04, 1.02, 0.99, 0.95, 0.97, 0.94, 0.96, 0.95, 0.94, 0.93, 0.92, 0.9, 0.9, 0.95, 0.99, 0.96, 0.98, 0.99, 0.95, 0.95, 0.95, 0.95, 0.91, 0.95, 0.96, 0.93, 0.92, 0.94, 0.91, 0.94, 0.94, 0.96, 0.94, 0.9, 0.9, 0.96, 0.95, 0.9, 0.92, 0.92, 0.96, 0.95, 0.93, 0.94, 0.97, 0.94, 0.93, 0.97, 0.94, 0.96, 0.96, 0.96, 0.93, 0.93, 0.94, 0.93, 0.93, 1.0, 0.95, 0.96, 0.96, 0.92, 0.96, 0.97, 0.98, 0.95, 0.93, 0.96, 0.94, 0.95, 0.98, 0.92, 0.95, 0.92, 0.94, 0.91, 0.94, 0.96, 0.94, 0.95, 0.91, 0.94, 0.94, 0.98, 0.96, 0.91, 0.91, 0.96, 0.96, 0.96, 0.95, 0.96, 0.93, 0.96, 0.95, 0.95, 0.92, 1.02, 0.96, 0.94, 0.95, 1.0, 0.99, 0.91, 0.98, 0.98, 1.0, 0.95, 0.99, 0.94, 0.94, 0.94, 0.96, 0.96, 0.92, 0.93, 0.96, 0.94, 0.94, 0.91, 0.9, 0.92, 0.92, 0.94, 0.89, 0.94, 0.92, 0.9, 0.93, 0.92, 0.89, 0.91, 0.92, 0.92, 0.89, 0.88, 0.92, 0.9, 0.9, 0.86, 0.91, 0.92, 0.87, 0.9, 0.94, 0.86, 0.89, 0.9, 0.94, 0.92, 0.96, 0.96, 0.91, 0.96, 0.91, 0.87, 0.96, 0.9, 0.99, 0.94, 0.91, 0.95, 0.9, 0.91, 0.95, 0.87, 0.99, 0.94, 0.84, 0.86, 0.91, 0.97, 0.91, 0.91, 0.95, 0.98, 0.98, 0.9, 0.93, 0.92, 0.92, 0.96, 0.98, 0.9, 0.91, 0.96, 0.96, 0.88, 0.88, 0.96, 0.96, 0.91, 0.91, 0.92, 0.92, 0.97, 0.97, 0.95, 0.95, 1.11, 1.11, 0.9, 0.9, 1.01, 1.01, 0.95, 0.95, 0.98, 0.98, 0.93, 0.93, 0.89, 0.87, 0.9, 0.91, 0.84, 0.9, 0.95, 0.9, 1.0, 0.96, 0.93, 0.92, 0.88, 0.87, 0.9, 0.94, 0.92, 0.84, 0.93, 0.88, 0.9, 0.93, 0.94, 0.85, 0.91, 0.9, 0.98, 0.9, 0.9, 0.98, 0.92, 0.91, 0.92, 0.92, 0.92, 0.9, 0.93, 0.93, 0.9, 0.92, 0.9, 0.95, 0.89, 0.85, 0.89, 0.87, 0.88, 0.95, 0.96, 0.82, 0.83, 0.91, 1.03, 0.91, 0.9, 0.94, 0.89, 0.91, 0.92, 1.02, 0.98, 0.96, 0.93, 0.91, 0.88, 0.86, 0.83, 0.93, 0.87, 0.88, 0.87, 0.92, 0.92, 0.96, 0.92, 0.83, 0.91, 0.89, 0.86, 0.9, 0.84, 0.83, 0.85, 0.85, 0.86, 0.85, 0.89, 0.93, 0.83, 0.92, 0.92, 0.9, 0.92, 0.79, 0.88, 0.82, 0.82, 0.91, 0.91, 0.94, 0.94, 0.9, 0.92, 0.83, 0.91, 0.85, 0.81, 0.88, 0.92, 0.84, 0.91, 0.86, 0.86, 0.92, 0.85, 0.95, 0.9, 0.9, 0.93, 0.86, 0.88, 0.83, 0.86, 0.92, 0.92, 0.92, 0.99, 0.89, 0.9, 0.88, 0.89, 0.92, 0.95, 0.91, 0.92, 0.88, 0.91, 0.92, 0.86, 0.91, 0.97, 0.96, 0.9, 0.83, 0.85, 0.87, 0.87, 0.98, 0.92, 0.91, 0.9, 0.91, 0.95, 0.95, 0.88, 0.88, 0.91, 0.86, 0.86, 0.89, 0.91, 0.9, 0.91, 0.91, 0.9, 0.89, 0.91, 0.91, 0.93, 0.93, 0.91, 0.91, 0.92, 0.92, 0.9, 0.88, 0.9, 0.89, 0.88, 0.85, 0.82, 0.85, 0.88, 0.92, 0.91, 0.96, 0.92, 0.94, 0.93, 0.92, 0.89, 0.92, 0.92, 0.9, 0.95, 0.93, 0.91, 0.93, 0.96, 0.97, 1.0, 0.96, 1.0, 0.96, 0.92, 0.94, 0.9, 0.92, 0.95, 0.91, 0.9, 0.91, 0.89, 0.9, 0.92, 0.91, 0.91, 0.92, 0.94, 0.9, 0.92, 0.92, 0.91, 0.94, 0.96, 0.92, 0.97, 0.95, 0.92, 0.92, 0.96, 0.92, 0.9, 0.98, 0.98, 0.94, 0.94, 0.9, 0.9, 0.9, 0.9, 0.98, 0.98, 1.04, 1.04, 0.98, 0.99, 0.94, 0.91, 1.0, 0.94, 0.91, 0.93, 0.9, 0.9, 0.96, 0.92, 0.92, 0.95, 0.91, 0.96, 0.9, 0.92, 0.96, 0.9, 0.97, 0.96, 0.92, 0.98, 0.96, 0.92, 0.96, 0.96, 1.03, 0.98, 1.04, 1.04, 0.96, 1.0, 0.95, 0.97, 0.99, 0.93, 1.0, 1.03, 0.93, 1.03, 0.91, 0.88, 0.92, 0.9, 1.03, 0.96, 0.96, 1.01, 0.87, 0.97, 0.92, 0.89, 0.92, 0.89, 0.96, 0.93, 0.91, 0.96, 0.9, 0.83, 0.9, 0.89, 0.99, 0.9, 0.91, 0.92, 0.92, 0.9, 0.9, 0.97, 0.93, 0.96, 0.93, 0.9, 0.92, 0.92, 0.93, 0.93, 0.9, 0.9, 0.92, 0.92, 0.9, 0.9, 0.91, 0.91, 0.98, 0.98, 1.03, 1.03, 0.97, 0.97, 0.92, 0.92, 0.88, 0.88, 0.95, 0.95, 0.85, 0.85, 0.9, 0.9, 1.01, 1.01, 0.91, 0.91, 1.05, 1.05, 0.98, 0.94, 1.01, 0.94, 0.92, 0.95, 0.98, 0.94, 0.99, 0.97, 0.91, 1.0, 1.01, 0.97, 0.96, 0.98, 0.93, 1.01, 0.95, 1.02, 0.96, 0.96, 0.93, 1.01, 0.96, 0.92, 0.88, 0.85, 0.94, 0.86, 0.91, 0.91, 0.92, 0.96, 0.9, 0.95, 0.95, 0.94, 1.0, 0.89, 0.91, 0.89, 0.99, 0.93, 0.96, 0.99, 1.0, 1.03, 0.9, 0.92, 0.9, 0.99, 0.9, 0.92, 0.92, 0.92, 0.94, 0.89, 0.9, 0.91, 0.93, 0.95, 0.93, 0.92, 0.88, 0.92, 0.9, 0.89, 1.01, 0.94, 0.87, 0.87, 0.89, 0.85, 0.86, 0.91, 0.99, 0.9, 0.88, 0.91, 0.91, 0.89, 0.93, 0.86, 0.85, 0.87, 0.94, 0.88, 0.89, 0.92, 0.89, 0.9, 0.86, 0.86, 0.89, 0.85, 0.87, 0.81, 0.89, 0.85, 0.83, 0.88, 0.9, 0.88, 0.89, 0.86, 0.86, 0.89, 0.92, 0.84, 0.86, 0.87, 0.88, 0.88, 0.9, 0.86, 0.88, 0.89, 0.9, 0.91, 0.86, 0.84, 0.86, 0.84, 0.87, 0.89, 0.87, 0.87, 0.86, 0.9, 0.92, 0.84, 0.92, 0.85, 0.85, 0.9, 0.92, 0.89, 0.9, 0.91, 0.9, 0.91, 0.9, 0.9, 0.93, 0.9, 0.9, 0.89, 0.92, 0.92, 0.9, 0.91, 0.91, 0.93, 0.9, 0.9, 0.92, 0.91, 0.91, 0.94, 0.91, 0.91, 0.93, 0.92, 0.9, 0.92, 0.92, 0.92, 0.9, 0.92, 0.9, 0.91, 0.92, 0.91, 0.93, 0.91, 0.9, 0.93, 0.95, 0.91, 0.93, 0.93, 0.93, 0.93, 0.96, 0.93, 0.9, 0.9, 0.91, 0.92, 0.9, 0.9, 0.92, 0.91, 0.9, 0.94, 0.9, 0.92, 0.92, 0.92, 0.9, 0.93, 0.94, 0.93, 0.95, 0.92, 0.93, 0.93, 0.95, 0.94, 0.91, 0.9, 0.91, 0.93, 0.9, 0.91, 0.9, 0.92, 0.92, 0.94, 0.96, 0.9, 0.92, 0.92, 0.91, 0.92, 0.91, 0.93, 0.92, 0.92, 0.9, 0.92, 0.92, 0.91, 0.94, 0.91, 0.94, 0.91, 0.89, 0.93, 0.93, 0.9, 0.9, 0.92, 0.92, 0.9, 0.93, 0.89, 0.89, 0.92, 0.92, 0.91, 0.9, 0.86, 0.91, 0.9]