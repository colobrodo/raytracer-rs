use std::fmt;

use super::{
    camera::Camera,
    core::{Light, Material, MaterialType, Scene, SceneObject, Solid},
    Mat4, Vec3,
};

pub struct SceneParser<'a> {
    content: &'a str,
    buffer: String,
    position: FilePosition,
}

#[derive(Debug, Clone, Copy)]
struct FilePosition {
    line: u32,
    column: u32,
    index: u32,
}

impl FilePosition {
    fn new() -> Self {
        FilePosition {
            line: 0,
            column: 0,
            index: 0,
        }
    }

    fn on_new_line(self: &mut Self) {
        self.line += 1;
        self.column = 0;
        self.index += 1;
    }

    fn advance(self: &mut Self) {
        self.column += 1;
        self.index += 1;
    }
}

#[derive(Debug)]
pub struct ParserError {
    position: FilePosition,
    pub message: String,
}

impl ParserError {
    fn new(message: &str, position: FilePosition) -> ParserError {
        ParserError {
            position,
            message: message.to_string(),
        }
    }

    pub fn print_error_location(self: &Self, content: &str) {
        println!("{}", self);
        if let Some(line) = content.lines().nth(self.position.line as usize) {
            println!("{}", line);
            let spacing = " ".repeat(self.position.column as usize);
            println!("{}^", spacing);
        }
    }
}

impl fmt::Display for ParserError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{} at {}:{}",
            self.message, self.position.line, self.position.column
        )
    }
}

type ParserResult<T> = Result<T, ParserError>;

pub struct ImageData {
    pub width: u32,
    pub height: u32,
    pub camera: Camera,
    pub scene: Scene,
}

// TODO: report error

impl SceneParser<'_> {
    pub fn new<'a>(content: &'a str) -> SceneParser<'a> {
        SceneParser {
            content,
            position: FilePosition::new(),
            buffer: "".to_string(),
        }
    }

    fn get_current_char(self: &Self) -> Option<char> {
        self.content.chars().nth(self.position.index as usize)
    }

    fn is_empty(self: &Self) -> bool {
        self.get_current_char().is_none()
    }

    fn advance(self: &mut Self) -> bool {
        if let Some(current_char) = self.get_current_char() {
            if current_char == '\n' {
                self.position.on_new_line();
            } else {
                self.position.advance();
            }
            return true;
        }
        return false;
    }

    fn advance_until(self: &mut Self, f: impl Fn(char) -> bool) {
        while let Some(current_char) = self.get_current_char() {
            if f(current_char) {
                break;
            }
            self.advance();
        }
    }

    fn eat_spaces(self: &mut Self) {
        // consume all the empty lines, spaces and comments before the next token
        while let Some(current_char) = self.get_current_char() {
            // comments
            if current_char == '#' {
                // consume the characters until the end of the line
                // note: we don't consume the end-of-line here but at the end of the loop
                self.advance_until(|c| c == '\n');
            } else if !current_char.is_whitespace() {
                break;
            }
            self.advance();
        }
    }

    fn pop(self: &mut Self) -> String {
        // check if we already peeked without eating the next token
        if !self.buffer.is_empty() {
            // TODO: should be possible to move to without clone first?
            let result = self.buffer.clone();
            self.buffer.clear();
            return result;
        }

        self.eat_spaces();
        let mut result = String::new();
        if self.is_empty() {
            return result;
        }
        let mut current_char = self.get_current_char().unwrap();
        // add the current char to the result string and advance
        let enqueque = move |parser: &mut SceneParser, result: &mut String| {
            if let Some(current_char) = parser.get_current_char() {
                result.push(current_char);
                parser.advance();
            }
            if let Some(next_char) = parser.get_current_char() {
                return next_char;
            }
            return ' ';
        };

        match current_char {
            // if char is a symbol return it
            ',' | '(' | ')' | ':' | '>' => {
                self.advance();
                result.push(current_char);
            }
            '"' => {
                enqueque(self, &mut result);
                // do not handle escape for now
                let mut in_string = true;
                loop {
                    current_char = enqueque(self, &mut result);
                    // eat also the last quote of the string
                    if !in_string {
                        break;
                    }
                    in_string = current_char != '"';
                }
            }
            // float parsing
            '.' | '+' | '-' | '0' | '1' | '2' | '3' | '4' | '5' | '6' | '7' | '8' | '9' => {
                if current_char == '+' || current_char == '-' {
                    current_char = enqueque(self, &mut result);
                }

                while current_char.is_digit(10) {
                    current_char = enqueque(self, &mut result);
                }

                if current_char == '.' {
                    current_char = enqueque(self, &mut result);
                    // TODO: here we should check if we already have encountred a digit before the dot
                    //       cause '.' is a valid float for now
                    while current_char.is_digit(10) {
                        current_char = enqueque(self, &mut result);
                    }
                }
            }

            _ => {
                while current_char.is_alphabetic() {
                    current_char = enqueque(self, &mut result);
                }
            }
        }
        return result;
    }

    fn peek(self: &mut Self) -> &String {
        // peek always look ahead and save the result to the buffer
        if self.buffer.is_empty() {
            self.buffer = self.pop();
        }
        return &self.buffer;
    }

    fn error<T>(self: &mut Self, message: &str) -> ParserResult<T> {
        Err(ParserError::new(message, self.position))
    }

    fn parse_float(self: &mut Self) -> ParserResult<f64> {
        let next_token = self.pop();
        // TODO: return a Result with ParserError and do not println hard
        if let Ok(num) = next_token.parse::<f64>() {
            Ok(num)
        } else {
            let message = format!(
                "error parsing file: cannot interp '{}' as a float",
                next_token
            );
            self.error(&message)
        }
    }

    fn match_token(self: &mut Self, expected_lexem: &str) -> ParserResult<()> {
        // match primitive: consume a lexem from the list and if is different
        // from the expected one raise an error
        let next_lexem = self.pop();
        if next_lexem != expected_lexem {
            // TODO: return parser error
            let message = format!(
                "error parsing the scene file: expected '{}', getting '{}' instead",
                expected_lexem, next_lexem
            );
            self.error(&message)
        } else {
            Ok(())
        }
    }

    fn maybe_match(self: &mut Self, expected_lexem: &str) -> bool {
        // variant of match that can fail
        // if the expected lexem is the next in the stream, we consume it and returns true.
        // return false otherwise leaving the stream untouched
        let next_lexem = self.peek();
        if *next_lexem == expected_lexem {
            self.pop();
            return true;
        }
        return false;
    }

    fn parse_header(self: &mut Self) -> ParserResult<(f64, f64)> {
        // TODO: unused for now
        self.match_token("size")?;
        let width = self.parse_float()?;
        let height = self.parse_float()?;
        Ok((width, height))
    }

    // TODO: substitute return value with Vec3
    fn parse_vec3(self: &mut Self) -> ParserResult<Vec3> {
        self.match_token("(")?;
        let x = self.parse_float()?;
        self.match_token(",")?;
        let y = self.parse_float()?;
        self.match_token(",")?;
        let z = self.parse_float()?;
        self.match_token(")")?;
        return Ok(Vec3::new(x, y, z));
    }

    // TODO: substitute return value with Vec3
    fn parse_color(self: &mut Self) -> ParserResult<Vec3> {
        // predefined color
        if self.maybe_match("red") {
            Ok(Vec3::new(1.0, 0.0, 0.0))
        } else if self.maybe_match("blue") {
            Ok(Vec3::new(0.0, 0.0, 1.0))
        } else if self.maybe_match("green") {
            Ok(Vec3::new(0.0, 1.0, 0.0))
        } else if self.maybe_match("white") {
            Ok(Vec3::new(1.0, 1.0, 1.0))
        } else if self.maybe_match("black") {
            Ok(Vec3::new(0.0, 0.0, 0.0))
        } else if self.maybe_match("cyan") {
            Ok(Vec3::new(0.0, 1.0, 01.0))
        } else if self.maybe_match("violet") {
            Ok(Vec3::new(1.0, 0.0, 1.0))
        } else if self.maybe_match("fuchsia") {
            Ok(Vec3::new(0.96, 0.0, 96.0))
        } else if self.maybe_match("yellow") {
            Ok(Vec3::new(1.0, 1.0, 0.0))
        } else if self.maybe_match("orange") {
            Ok(Vec3::new(0.98, 0.45, 0.02))
        } else {
            self.parse_vec3()
        }
    }

    fn parse_material(self: &mut Self) -> ParserResult<Material> {
        let mut material_type = MaterialType::Plastic;
        if self.maybe_match("metal") {
            material_type = MaterialType::Metal;
            self.match_token(":")?;
        } else if self.maybe_match("plastic") {
            material_type = MaterialType::Metal;
            self.match_token(":")?;
        };
        let color = self.parse_color()?;
        Ok(Material {
            color,
            type_: material_type,
        })
    }

    fn parse_sphere(self: &mut Self) -> ParserResult<SceneObject> {
        self.match_token("sphere")?;
        let center = self.parse_vec3()?;
        let radius = self.parse_float()?;
        let material = self.parse_material()?;
        Ok(SceneObject {
            solid: Solid::Sphere { center, radius },
            material,
        })
    }

    fn parse_plane(self: &mut Self) -> ParserResult<SceneObject> {
        self.match_token("plane")?;
        let normal = self.parse_vec3()?;
        let distance = self.parse_float()?;
        let material = self.parse_material()?;
        Ok(SceneObject {
            solid: Solid::Plane { normal, distance },
            material,
        })
    }

    fn parse_string(self: &mut Self) -> ParserResult<String> {
        let mut next_token = self.pop();
        // TODO: return parse error when the popped token don't end and starts with a quote
        // remove quotes
        next_token.drain(..1);
        next_token.drain(next_token.len() - 1..);
        Ok(next_token)
    }

    fn parse_camera(&mut self) -> ParserResult<Camera> {
        if self.maybe_match("camera") {
            let mut position = Vec3::zero();
            if self.maybe_match("from") {
                position = self.parse_vec3()?;
            }
            let point = if self.maybe_match("to") {
                self.parse_vec3()?
            } else {
                position + Vec3::z_axis()
            };
            Ok(Camera::look_at(position, point))
        } else {
            Ok(Camera::default())
        }
    }

    fn parse_trasformation(self: &mut Self) -> ParserResult<Mat4> {
        let mut trasform = Mat4::identity();
        while self.maybe_match(">") {
            let next_token = self.peek();
            let next_trasform = match next_token.as_str() {
                "scale" => {
                    self.pop(); // discarding "scale"
                    let factor = self.parse_float()?;
                    Mat4::scale(factor)
                }
                "translate" => {
                    self.pop(); // discarding "translate"
                    let offset = self.parse_vec3()?;
                    Mat4::translate(offset)
                }
                "rotate" => {
                    self.pop(); // discarding "rotate"
                    let axis = self.parse_vec3()?;
                    let angle = self.parse_float()?;
                    Mat4::rotate(axis, angle)
                }
                _ => return self.error("unexpected token while parsing trasform"),
            };
            trasform = trasform.then(&next_trasform);
        }
        Ok(trasform)
    }

    fn parse_model(self: &mut Self) -> ParserResult<SceneObject> {
        self.match_token("model")?;
        let path = self.parse_string()?;
        let material = self.parse_material()?;
        let trasform = self.parse_trasformation()?;
        Solid::load_model(&path, trasform)
            .map_err(|err| {
                let message = format!("Cannot load model  \"{}\"", path);
                println!("{}", err);
                self.error::<Solid>(&message).expect_err("")
            })
            .map(|model| SceneObject {
                material,
                solid: model,
            })
    }

    fn parse_light(self: &mut Self) -> ParserResult<Light> {
        self.match_token("light")?;
        let position = self.parse_vec3()?;
        let color = self.parse_color()?;
        return Ok(Light {
            position,
            color,
            radius: 2.0,
        });
    }

    pub fn parse_scene(self: &mut Self) -> ParserResult<ImageData> {
        // main routine that parse the whole file
        let (width, height) = self.parse_header()?;
        let camera = self.parse_camera()?;

        let mut objects = Vec::new();
        let mut lights = Vec::new();
        while !self.is_empty() {
            let next_token = self.peek();
            match next_token.as_str() {
                "light" => {
                    let light = self.parse_light()?;
                    lights.push(light);
                }
                "sphere" => {
                    let object = self.parse_sphere()?;
                    objects.push(object);
                }
                "plane" => {
                    let object = self.parse_plane()?;
                    objects.push(object);
                }
                "model" => {
                    let object = self.parse_model()?;
                    objects.push(object);
                }
                _ => {
                    let message = format!("unexpected token '{}'", next_token);
                    return self.error(&message);
                }
            }
        }
        let scene = Scene { objects, lights };
        Ok(ImageData {
            width: width as u32,
            height: height as u32,
            camera,
            scene,
        })
    }
}
