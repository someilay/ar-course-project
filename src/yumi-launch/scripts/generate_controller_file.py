import argparse


def generate_controller_file(template_path: str, kwargs: dict[str, str]) -> str:
    with open(template_path, "r") as f:
        template = f.read()
    return template.format(**kwargs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t",
        "--template_path",
        action="store",
        type=str,
        required=True,
    )
    parser.add_argument(
        "-v",
        "--values",
        nargs="+",
        type=str,
        required=True,
    )
    args = parser.parse_args()
    values = [x.split("=") for x in args.values]

    file_content = generate_controller_file(
        args.template_path, {key: val for key, val in values},
    )
    print(file_content)


if __name__ == "__main__":
    main()
