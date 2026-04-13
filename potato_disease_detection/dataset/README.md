## Dataset Preparation

1. **Download the Dataset**  
   Get the dataset from [Kaggle – Plant Disease](https://www.kaggle.com/datasets/emmarex/plantdisease).

2. **Create the Directory Structure**  
   Create folders for training and validation data:
   ```bash
   mkdir -p train/Early_Blight train/Healthy train/Late_Blight \
            val/Early_Blight   val/Healthy   val/Late_Blight
3. **Use the provided Python script to split the dataset into training and validation sets:**
    ```bash
    python3 split_dataset.py
    ```
    Ensure to edit to match your path