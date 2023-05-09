from  PIL import  Image
from lang_sam import LangSAM
from lang_sam.utils import draw_image
import numpy as np

model = LangSAM(sam_type="vit_b")
image_pil = Image.open('src/python/segmentation/grocery_tomato.png').convert("RGB")
np_image = np.array(Image.open('src/python/segmentation/grocery_tomato.png'))
text_prompt = 'canned tomato'
masks, boxes, phrases, logits = model.predict(image_pil, text_prompt)

print("Masks", masks)
print("Boxes", boxes)
print("Phrases", phrases)
print("Logits", logits)

result = draw_image(np_image, masks, boxes, phrases)
print("Result")
print(result)
final_result = Image.fromarray(result)
final_result.save('src/python/segmentation/grocery_tomato_result.jpeg', 'JPEG')
